"""Tier 3 Pairwise Integration Tests: LiDAR Sensor ↔ Optical BBox Fallback (R4 + R3).

Verifies seamless transition from LiDAR range measurements to passive optical bounding box
ranging upon sensor disconnect or communication loss without dropping telemetry to None.
"""

from __future__ import annotations

import pytest

from tests.fixtures.mock_serial import MockSerialPort, TFMiniPacketGenerator
from tests.fixtures.synthetic_video import TargetKinematics
from tests.tier1_features.test_lidar_parser import TFMiniLidarParser
from tests.tier1_features.test_bbox_distance import DistanceEstimator


def test_lidar_to_optical_fallback_seamless_transition(mock_serial, tfmini_gen):
    """T3.2.1: Verifies continuous distance telemetry when LiDAR disconnects mid-tracking."""
    parser = TFMiniLidarParser()
    estimator = DistanceEstimator(focal_length=800.0, preset="dji_mavic", ema_alpha=0.5)

    target = TargetKinematics(x=0.0, y=0.0, z=20.0, real_size=0.40)
    # Bbox for 20m: width = 800 * 0.40 / 20 = 16px -> (312, 232, 328, 248)
    bbox = target.project_to_camera(800.0, 320.0, 240.0)

    distance_history = []
    sources = []

    # Phase 1: 20 frames with active LiDAR (20.0m)
    for _ in range(20):
        mock_serial.inject_tfmini_packet(distance_cm=2000, strength=1000)
        parsed = parser.feed_bytes(mock_serial.read(64))
        lidar_val = parsed[-1] if parsed else None

        dist, src = estimator.get_distance(bbox, lidar_dist=lidar_val)
        distance_history.append(dist)
        sources.append(src)

    assert all(s == "lidar" for s in sources[:20])
    assert distance_history[-1] == pytest.approx(20.0, abs=0.5)

    # Phase 2: LiDAR disconnects (no new packets injected; feed_bytes returns [])
    for _ in range(20):
        parsed = parser.feed_bytes(mock_serial.read(64))
        lidar_val = parsed[-1] if parsed else None

        dist, src = estimator.get_distance(bbox, lidar_dist=lidar_val)
        distance_history.append(dist)
        sources.append(src)

    # Telemetry should seamlessly switch to optical
    assert all(s == "optical" for s in sources[20:])
    # Distance should remain steady near ~20.0m without step jump or dropping to 0
    assert all(18.0 <= d <= 22.0 for d in distance_history)


def test_lidar_corruption_recovery(mock_serial, tfmini_gen):
    """T3.2.2: Verifies optical fallback handles intermittent corrupt LiDAR packets."""
    parser = TFMiniLidarParser()
    estimator = DistanceEstimator(focal_length=800.0, preset="dji_mavic")
    bbox = (300, 220, 340, 260)  # Optical = 8.0m (40px)

    # Inject corrupted checksum packet
    mock_serial.inject_rx(tfmini_gen.build_corrupted_checksum_packet(distance_cm=1200))
    parsed = parser.feed_bytes(mock_serial.read(64))
    lidar_val = parsed[-1] if parsed else None

    # Corrupt packet rejected -> falls back to optical (8.0m)
    dist, src = estimator.get_distance(bbox, lidar_dist=lidar_val)
    assert src == "optical"
    assert dist == pytest.approx(8.0)
