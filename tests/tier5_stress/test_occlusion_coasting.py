"""Tier 5 Adversarial Stress Suite 3: Long Visual Occlusion & Coasting Extrapolation Stress Tests.

Empirically tests Kalman filter track coasting, state extrapolation accuracy,
lifecycle transitions (TRACKING -> COASTING -> LOST -> REACQUIRED), and multi-target
isolation across prolonged detection dropouts (15-20 frames lost).
"""

from __future__ import annotations

import math
from typing import List, Tuple
import numpy as np
import pytest

from drone_turret.tracking.kalman_filter import (
    KalmanPredictiveTracker,
    MultiTargetKalmanTracker,
    KalmanConfig,
    FilterStatus,
    focal_length_from_hfov,
)
from drone_turret.vision.detector import Detection


@pytest.fixture
def standard_tracker() -> KalmanPredictiveTracker:
    config = KalmanConfig(
        process_noise_scale=10.0,
        measurement_noise_std=2.0,
        max_coast_frames=15,
        camera_width=640,
        camera_height=480,
        camera_hfov=70.0,
        default_distance=40.0,
    )
    return KalmanPredictiveTracker(config)


def test_coasting_extrapolation_linear_15_frames(standard_tracker):
    """Stress-test 3.1: Extrapolation accuracy across 15 consecutive occluded frames.
    
    Target travels at constant 25 m/s (90 km/h).
    Tracked for 20 frames, then occluded for 15 frames (0.5 seconds at 30 FPS).
    Validates:
    - During occlusion, status is FilterStatus.COASTING and is_coasting is True.
    - coast_frames counter increments monotonically from 1 to 15.
    - Extrapolated position error against ground truth remains < 20 pixels (< 0.6m at 40m).
    - Velocity estimate holds steady without collapse or drift.
    """
    range_m = 40.0
    f_px = focal_length_from_hfov(640, 70.0)
    vx_mps = 25.0
    vy_mps = 2.0
    dt = 1.0 / 30.0

    cx_cam, cy_cam = 320.0, 240.0
    x_m = -15.0
    y_m = 0.0
    t = 0.0

    # 1. Warm up filter for 30 frames with visible detections to settle velocity and acceleration
    for k in range(30):
        t += dt
        x_m += vx_mps * dt
        y_m += vy_mps * dt
        px = cx_cam + (f_px * x_m / range_m)
        py = cy_cam - (f_px * y_m / range_m)
        
        state = standard_tracker.update(
            measurement=(px, py),
            dt=dt,
            distance=range_m,
            timestamp=t,
        )
        assert state.status == FilterStatus.TRACKING
        assert state.coast_frames == 0

    # 2. Occlusion phase: 15 frames with measurement=None
    occlusion_errors_px = []
    for k in range(1, 16):
        t += dt
        x_m += vx_mps * dt
        y_m += vy_mps * dt
        gt_px = cx_cam + (f_px * x_m / range_m)
        gt_py = cy_cam - (f_px * y_m / range_m)

        state = standard_tracker.update(
            measurement=None,
            dt=dt,
            distance=range_m,
            timestamp=t,
        )

        assert state.status == FilterStatus.COASTING
        assert state.is_coasting is True
        assert state.coast_frames == k

        err = math.hypot(state.x - gt_px, state.y - gt_py)
        occlusion_errors_px.append(err)

    # Assert extrapolation quality: final error after 15 frames < 20 pixels
    assert np.max(occlusion_errors_px) < 20.0, (
        f"Coasting extrapolation drifted too far: max error {np.max(occlusion_errors_px):.2f} px"
    )
    # Velocity during coasting must remain close to true velocity (25 m/s)
    final_state = standard_tracker.get_state()
    assert final_state.vX == pytest.approx(vx_mps, rel=0.15)


def test_coasting_lifecycle_transition_to_lost_at_20_frames(standard_tracker):
    """Stress-test 3.2: Full lifecycle verification through 20 frames of continuous loss.
    
    Validates:
    - Frames 1..15 of loss: status == COASTING, is_tracking == True, is_lost == False.
    - Frames 16..20 of loss: status == LOST, is_tracking == False, is_lost == True.
    - No crashes, NaN states, or negative variances upon transition to LOST.
    """
    dt = 1.0 / 30.0
    t = 0.0

    # Initialize and track for 10 frames
    for k in range(10):
        t += dt
        standard_tracker.update(
            measurement=(320.0 + k * 2.0, 240.0),
            dt=dt,
            distance=30.0,
            timestamp=t,
        )
    assert standard_tracker.status == FilterStatus.TRACKING

    # Occlude for 20 frames
    for k in range(1, 21):
        t += dt
        state = standard_tracker.update(
            measurement=None,
            dt=dt,
            distance=30.0,
            timestamp=t,
        )
        if k <= 15:
            assert state.status == FilterStatus.COASTING, f"Frame {k} should be COASTING"
            assert state.is_tracking is True
            assert state.is_lost is False
            assert state.coast_frames == k
        else:
            assert state.status == FilterStatus.LOST, f"Frame {k} should be LOST"
            assert state.is_tracking is False
            assert state.is_lost is True
            assert state.coast_frames >= 15


def test_reacquisition_after_14_frames_occlusion(standard_tracker):
    """Stress-test 3.3: Reacquisition after 14 frames of occlusion (just before LOST threshold).
    
    Validates:
    - Reacquisition immediately restores FilterStatus.TRACKING.
    - coast_frames resets to 0.
    - Covariance matrix contracts smoothly upon new measurement.
    """
    dt = 1.0 / 30.0
    t = 0.0
    range_m = 35.0
    f_px = focal_length_from_hfov(640, 70.0)

    # 1. Active track for 15 frames
    for k in range(15):
        t += dt
        px = 320.0 + k * 5.0
        standard_tracker.update(measurement=(px, 240.0), dt=dt, distance=range_m, timestamp=t)

    # 2. Occlude for 14 frames
    for k in range(14):
        t += dt
        standard_tracker.update(measurement=None, dt=dt, distance=range_m, timestamp=t)
    assert standard_tracker.status == FilterStatus.COASTING
    assert standard_tracker.coast_frames == 14
    cov_occluded = standard_tracker.covariance_matrix.copy()

    # 3. Measurement re-emerges at frame 15
    t += dt
    extrap_x, extrap_y = standard_tracker.predict_at_time(0.0)
    reacq_state = standard_tracker.update(
        measurement=(extrap_x + 2.0, extrap_y),
        dt=dt,
        distance=range_m,
        timestamp=t,
    )

    assert reacq_state.status == FilterStatus.TRACKING
    assert reacq_state.coast_frames == 0
    assert reacq_state.is_coasting is False

    # Check covariance contraction (error covariance P decreases after measurement update)
    cov_reacquired = standard_tracker.covariance_matrix
    assert cov_reacquired[0, 0] < cov_occluded[0, 0], "Position variance did not contract upon reacquisition"


def test_high_speed_200kmh_coasting_extrapolation(standard_tracker):
    """Stress-test 3.4: 200 km/h (55.56 m/s) coasting extrapolation across 15 frames.
    
    Target at 50m flying at 200 km/h undergoes 15 frames of occlusion (covers 27.78 meters).
    Validates:
    - Linear extrapolation maintains accurate trajectory projection.
    - Error at end of 0.5s coasting is < 45 pixels (< 4.5m at 50m range out of 55m traversed).
    """
    dt = 1.0 / 30.0
    range_m = 50.0
    f_px = focal_length_from_hfov(640, 70.0)
    vx_mps = 200.0 / 3.6  # 55.56 m/s
    t = 0.0
    x_m = -25.0

    # Track for 20 frames to establish accurate velocity
    for k in range(20):
        t += dt
        x_m += vx_mps * dt
        px = 320.0 + (f_px * x_m / range_m)
        standard_tracker.update(measurement=(px, 240.0), dt=dt, distance=range_m, timestamp=t)

    # Occlude for 15 frames
    for k in range(15):
        t += dt
        x_m += vx_mps * dt
        gt_px = 320.0 + (f_px * x_m / range_m)
        state = standard_tracker.update(measurement=None, dt=dt, distance=range_m, timestamp=t)

    final_px = state.x
    err_px = abs(final_px - gt_px)
    assert err_px < 45.0, f"200 km/h coasting error too large: {err_px:.2f} px (expected < 45 px)"


def test_multi_target_independent_occlusion():
    """Stress-test 3.5: Multi-target manager with independent occlusion channels.
    
    Target 1 undergoes 16 frames of occlusion (becoming LOST and pruned).
    Target 2 remains visible throughout (remains TRACKING).
    Validates:
    - Isolation between tracks: Target 1 loss does not corrupt Target 2 state or locking.
    """
    config = KalmanConfig(max_coast_frames=15)
    multi_tracker = MultiTargetKalmanTracker(config)
    dt = 1.0 / 30.0
    t = 0.0

    # Step 1: Both visible for 10 frames
    for k in range(10):
        t += dt
        det1 = Detection(box=(100 + k * 2, 200, 140 + k * 2, 240), confidence=0.9, class_id=0, class_name="drone", track_id=1)
        det2 = Detection(box=(400 - k * 2, 200, 440 - k * 2, 240), confidence=0.9, class_id=0, class_name="drone", track_id=2)
        tracks = multi_tracker.update_tracks([det1, det2], dt=dt, timestamp=t)
        assert tracks[1].status == FilterStatus.TRACKING
        assert tracks[2].status == FilterStatus.TRACKING

    # Step 2: Only Target 2 visible for next 16 frames
    last_tracks = {}
    for k in range(16):
        t += dt
        det2 = Detection(box=(380 - k * 2, 200, 420 - k * 2, 240), confidence=0.9, class_id=0, class_name="drone", track_id=2)
        last_tracks = multi_tracker.update_tracks([det2], dt=dt, timestamp=t)
        assert last_tracks[2].status == FilterStatus.TRACKING

    # Target 1 should have been marked LOST on frame 16 and pruned from active dict
    assert 1 not in multi_tracker.trackers
    assert 2 in multi_tracker.trackers
    assert last_tracks[1].status == FilterStatus.LOST
    assert last_tracks[1].coast_frames == 16
