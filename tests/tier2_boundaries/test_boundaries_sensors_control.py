"""
Tier 2 Boundary Tests: Edge Cases and Extreme Boundary Conditions for Sensors & Control.
"""

from __future__ import annotations

import math
import pytest

from drone_turret.comms.serial_comm import (
    MockSerialTransport,
    SerialCommunicator,
)
from drone_turret.control.pid import (
    DEFAULT_DEADBAND_DEG,
    DiscretePID,
    TurretController,
)
from drone_turret.sensors.distance import (
    DistanceEstimator,
    compute_focal_length_px,
    compute_hfov_deg,
    get_drone_size,
)
from drone_turret.sensors.lidar import (
    LidarParser,
    LidarReading,
)
from tests.fixtures.mock_serial import TFMiniPacketGenerator


# ============================================================================
# LiDAR Binary Parser Boundary Tests
# ============================================================================

def test_lidar_boundary_zero_and_max_distance():
    """Tests 0cm distance and 65535cm (0xFFFF) full 16-bit range."""
    parser = LidarParser(min_distance_m=0.1, max_distance_m=50.0)

    # 0 cm -> distance_m = 0.0 -> below min_dist -> invalid
    pkt_zero = TFMiniPacketGenerator.build_packet(distance_cm=0, strength=500)
    r_zero = parser.parse_packet(pkt_zero)
    assert r_zero is not None
    assert r_zero.distance_m == 0.0
    assert r_zero.is_valid is False

    # 65535 cm -> 655.35m -> above max_dist -> invalid
    pkt_max = TFMiniPacketGenerator.build_packet(distance_cm=65535, strength=500)
    r_max = parser.parse_packet(pkt_max)
    assert r_max is not None
    assert r_max.distance_m == 655.35
    assert r_max.is_valid is False


def test_lidar_boundary_truncated_and_empty_inputs():
    """Tests empty byte streams and truncated packets of length 0 to 8."""
    parser = LidarParser()
    
    assert parser.feed(b"") == []
    assert parser.parse_packet(b"") is None
    
    for length in range(1, 9):
        truncated = bytes([0x59] * length)
        assert parser.parse_packet(truncated) is None


def test_lidar_boundary_strength_exact_threshold():
    """Tests strength exactly at threshold (99 vs 100)."""
    parser = LidarParser(min_strength=100)
    
    pkt_99 = TFMiniPacketGenerator.build_packet(distance_cm=500, strength=99)
    r_99 = parser.parse_packet(pkt_99)
    assert r_99 is not None and r_99.is_valid is False

    pkt_100 = TFMiniPacketGenerator.build_packet(distance_cm=500, strength=100)
    r_100 = parser.parse_packet(pkt_100)
    assert r_100 is not None and r_100.is_valid is True


# ============================================================================
# Distance Estimator Boundary Tests
# ============================================================================

def test_distance_boundary_zero_and_negative_bbox_width():
    """Tests degenerate 0-width or negative coordinate bounding boxes."""
    estimator = DistanceEstimator(camera_hfov_deg=70.0)
    frame_shape = (480, 640)

    # Zero width: (100, 100, 100, 150) -> width = 0
    dist_zero = estimator.estimate_optical_distance(
        bbox=(100, 100, 100, 150),
        frame_shape=frame_shape,
    )
    assert dist_zero > 0
    assert dist_zero <= estimator.max_distance_m

    # Inverted coordinates: (200, 100, 150, 150) -> width = abs(150 - 200) = 50
    dist_inv = estimator.estimate_optical_distance(
        bbox=(200, 100, 150, 150),
        frame_shape=frame_shape,
    )
    assert dist_inv > 0


def test_distance_boundary_extreme_fov_and_resolutions():
    """Tests extreme field of view values and non-standard image resolutions."""
    # Near-zero HFOV
    f_narrow = compute_focal_length_px(640, hfov_deg=1.0)
    assert f_narrow > 10000.0

    # Ultra-wide 170° HFOV
    f_wide = compute_focal_length_px(640, hfov_deg=170.0)
    assert f_wide < 100.0

    # 4K UHD resolution (3840x2160)
    f_4k = compute_focal_length_px(3840, hfov_deg=70.0)
    assert pytest.approx(f_4k, abs=1.0) == 3840 / (2 * math.tan(math.radians(35.0)))


# ============================================================================
# PID Controller Boundary Tests
# ============================================================================

def test_pid_boundary_zero_and_negative_dt():
    """Tests robustness when dt <= 0.0."""
    pid = DiscretePID(kp=0.5, ki=0.1, kd=0.05, initial_angle=90.0)
    
    # dt = 0.0 should not crash or divide by zero
    out1 = pid.update(target_deg=120.0, dt=0.0)
    assert 90.0 <= out1 <= 180.0

    # dt < 0.0
    out2 = pid.update(target_deg=120.0, dt=-0.05)
    assert 90.0 <= out2 <= 180.0


def test_pid_boundary_nan_and_inf_target():
    """Tests immunity against NaN and Infinity targets."""
    pid = DiscretePID(initial_angle=90.0)
    
    out_nan = pid.update(target_deg=float("nan"), dt=0.033)
    assert out_nan == 90.0

    out_inf = pid.update(target_deg=float("inf"), dt=0.033)
    assert out_inf == 90.0


def test_pid_boundary_deadband_exact_threshold():
    """Tests exact edge around deadband threshold (0.3000° vs 0.3001°)."""
    pid = DiscretePID(kp=1.0, deadband=0.3, initial_angle=90.0)
    
    # Exact threshold: 90.3000° -> error = 0.3000° <= 0.3000° -> filtered error = 0
    out_exact = pid.update(target_deg=90.3, dt=0.033)
    assert out_exact == 90.0

    # Slightly above: 90.301° -> error = 0.301° > 0.3° -> output changes
    pid.reset(initial_angle=90.0)
    out_above = pid.update(target_deg=90.301, dt=0.033)
    assert out_above > 90.0


# ============================================================================
# Serial Comms Boundary Tests
# ============================================================================

def test_serial_boundary_queue_overflow_recovery():
    """Tests rapid queue flooding without dropped state or deadlock."""
    comm = SerialCommunicator(simulation_mode=True, tx_rate_hz=100.0)
    
    # Flood 200 commands into queue with maxsize=100
    for i in range(200):
        comm.send_angles(float(i % 180), float((180 - i) % 180), immediate=False)

    status = comm.get_status()
    assert status["connected"] is True
    
    comm.disconnect()
