"""Tier 1 Unit Feature Tests for Milestone M2: Kalman Predictive Tracking.

Features:
- F4: 6-State Constant Acceleration Kalman Filter (>= 5 tests)
- F5: Forward Trajectory Prediction 0.1 - 2.0s (>= 5 tests)
- F6: Target Loss / Occlusion Coasting >= 10-15 frames (>= 5 tests)
- F7: Speed Calculation in km/h and 3D kinematics (>= 5 tests)
"""

import math
import numpy as np
import pytest

from drone_turret.tracking import (
    FilterStatus,
    KalmanConfig,
    KalmanPredictiveTracker,
    TargetState,
    compute_process_noise,
    compute_transition_matrix,
    focal_length_from_hfov,
    pixel_to_camera_3d,
    velocity_to_speed_kmh,
)


class TestTier1FeatureF4KalmanFilter:
    """F4: 6-State Constant Acceleration Kalman Filter."""

    def test_f4_01_state_vector_dimensions_and_types(self):
        """F4.1: Verify state vector is 6-dimensional float32 and measurement is 2-dimensional."""
        tracker = KalmanPredictiveTracker()
        assert tracker.kf.statePost.shape == (6, 1)
        assert tracker.kf.statePre.shape == (6, 1)
        assert tracker.kf.measurementMatrix.shape == (2, 6)
        assert tracker.kf.processNoiseCov.shape == (6, 6)
        assert tracker.kf.measurementNoiseCov.shape == (2, 2)
        assert tracker.kf.errorCovPost.shape == (6, 6)

    def test_f4_02_dynamic_dt_state_transition(self):
        """F4.2: Verify dynamic dt recalculates F(dt) accurately for arbitrary time delta."""
        tracker = KalmanPredictiveTracker()
        tracker.initialize(initial_pos=(100.0, 100.0), initial_vel=(200.0, 100.0), initial_acc=(20.0, -10.0))
        
        # Test predict with dt=0.25s
        dt = 0.25
        tracker.predict(dt=dt)
        # Expected: x = 100 + 200*(0.25) + 0.5*20*(0.25^2) = 100 + 50 + 0.625 = 150.625
        # Expected: y = 100 + 100*(0.25) + 0.5*(-10)*(0.25^2) = 100 + 25 - 0.3125 = 124.6875
        state = tracker.get_state()
        assert state.pos_2d[0] == pytest.approx(150.625, rel=1e-4)
        assert state.pos_2d[1] == pytest.approx(124.6875, rel=1e-4)

    def test_f4_03_noisy_sinusoid_tracking(self):
        """F4.3: Filter smoothly tracks oscillatory motion with additive measurement noise."""
        tracker = KalmanPredictiveTracker(KalmanConfig(process_noise_scale=2000.0, measurement_noise_std=3.0))
        dt = 0.0333
        
        for frame in range(90):
            t = frame * dt
            meas_x = 320.0 + 100.0 * math.sin(2.0 * math.pi * 0.5 * t)
            meas_y = 240.0 + 50.0 * math.cos(2.0 * math.pi * 0.5 * t)
            # Add Gaussian noise
            noisy_x = meas_x + np.random.normal(0, 2.0)
            noisy_y = meas_y + np.random.normal(0, 2.0)
            state = tracker.update((noisy_x, noisy_y), dt=dt)

        # Tracked output should remain smooth and close to ground truth
        assert abs(state.pos_2d[0] - meas_x) < 20.0
        assert abs(state.pos_2d[1] - meas_y) < 20.0

    def test_f4_04_error_covariance_decay(self):
        """F4.4: Error covariance P decreases as consecutive valid measurements arrive."""
        tracker = KalmanPredictiveTracker()
        tracker.initialize(initial_pos=(100.0, 100.0))
        initial_p_trace = float(np.trace(tracker.covariance_matrix))

        for i in range(20):
            tracker.update((100.0 + i, 100.0), dt=0.033)

        final_p_trace = float(np.trace(tracker.covariance_matrix))
        assert final_p_trace < initial_p_trace

    def test_f4_05_explicit_timestamp_tracking(self):
        """F4.5: Elapsed dt is automatically calculated from monotonic frame timestamps."""
        tracker = KalmanPredictiveTracker()
        tracker.update((50.0, 50.0), timestamp=100.0)
        assert tracker.last_timestamp == 100.0

        state = tracker.update((70.0, 50.0), timestamp=100.1)  # dt = 0.1s
        assert tracker.last_timestamp == 100.1
        # vx estimated: ~ (70-50)/0.1 = 200 px/s
        assert state.vel_2d[0] > 50.0


class TestTier1FeatureF5TrajectoryProjection:
    """F5: Forward Trajectory Prediction (0.1 - 2.0s)."""

    def test_f5_01_linear_trajectory_points(self):
        """F5.1: Predict trajectory for constant linear velocity."""
        tracker = KalmanPredictiveTracker()
        tracker.initialize(initial_pos=(100.0, 200.0), initial_vel=(50.0, 0.0))

        traj = tracker.predict_trajectory(dt_horizon=1.0, num_steps=10)
        assert len(traj) == 11
        for i, (px, py) in enumerate(traj):
            t = i * 0.1
            assert px == pytest.approx(100.0 + 50.0 * t, abs=1e-3)
            assert py == pytest.approx(200.0, abs=1e-3)

    def test_f5_02_accelerated_parabolic_trajectory(self):
        """F5.2: Predict trajectory with non-zero acceleration forms a quadratic curve."""
        tracker = KalmanPredictiveTracker()
        tracker.initialize(
            initial_pos=(0.0, 0.0),
            initial_vel=(0.0, 100.0),
            initial_acc=(50.0, -20.0),
        )

        traj = tracker.predict_trajectory(dt_horizon=2.0, num_steps=20)
        assert len(traj) == 21
        # t = 1.0s: x = 0.5*50*(1^2) = 25, y = 100*(1) + 0.5*(-20)*(1^2) = 90
        t1_idx = 10  # 10 * (2.0/20) = 1.0s
        assert traj[t1_idx][0] == pytest.approx(25.0, abs=1e-3)
        assert traj[t1_idx][1] == pytest.approx(90.0, abs=1e-3)

    def test_f5_03_predict_at_arbitrary_future_time(self):
        """F5.3: predict_at_time evaluates point at arbitrary future continuous time t."""
        tracker = KalmanPredictiveTracker()
        tracker.initialize(initial_pos=(50.0, 50.0), initial_vel=(80.0, -40.0), initial_acc=(0.0, 10.0))

        px, py = tracker.predict_at_time(0.75)
        # x = 50 + 80*0.75 = 110
        # y = 50 - 40*0.75 + 0.5*10*(0.75^2) = 50 - 30 + 2.8125 = 22.8125
        assert px == pytest.approx(110.0, abs=1e-4)
        assert py == pytest.approx(22.8125, abs=1e-4)

    def test_f5_04_3d_forward_trajectory_projection(self):
        """F5.4: predict_trajectory_3d outputs 3D points (X, Y, Z) in meters."""
        tracker = KalmanPredictiveTracker()
        tracker.initialize(initial_pos=(320.0, 240.0), initial_vel=(0.0, 0.0), distance=40.0)

        traj_3d = tracker.predict_trajectory_3d(dt_horizon=1.5, num_steps=15)
        assert len(traj_3d) == 16
        for pt in traj_3d:
            assert len(pt) == 3
            assert pt[2] == pytest.approx(40.0, abs=1e-2)

    def test_f5_05_uninitialized_trajectory_is_empty(self):
        """F5.5: When uninitialized, predict_trajectory safely returns empty list."""
        tracker = KalmanPredictiveTracker()
        assert tracker.predict_trajectory(dt_horizon=1.0) == []
        assert tracker.predict_trajectory_3d(dt_horizon=1.0) == []


class TestTier1FeatureF6OcclusionCoasting:
    """F6: Target Loss / Occlusion Coasting (>= 10-15 frames)."""

    def test_f6_01_coasting_flags_and_counters(self):
        """F6.1: Coasting frame counter increments strictly by 1 per missed frame."""
        tracker = KalmanPredictiveTracker(KalmanConfig(max_coast_frames=15))
        tracker.initialize(initial_pos=(100.0, 100.0), initial_vel=(10.0, 0.0))

        for frame in range(1, 11):
            state = tracker.update(measurement=None, dt=0.033)
            assert state.coast_frames == frame
            assert state.is_coasting is True
            assert tracker.status == FilterStatus.COASTING

    def test_f6_02_lock_retention_over_12_frames(self):
        """F6.2: Target lock is maintained for >= 10-12 dropped detection frames."""
        tracker = KalmanPredictiveTracker(KalmanConfig(max_coast_frames=15))
        tracker.initialize(initial_pos=(200.0, 200.0), initial_vel=(50.0, 0.0))

        for _ in range(12):
            state = tracker.update(measurement=None, dt=0.033)
            assert tracker.is_tracking is True
            assert tracker.is_lost is False

    def test_f6_03_transition_to_lost_at_frame_16(self):
        """F6.3: State machine cleanly transitions to LOST when coast_frames > 15."""
        tracker = KalmanPredictiveTracker(KalmanConfig(max_coast_frames=15))
        tracker.initialize(initial_pos=(100.0, 100.0))

        for _ in range(15):
            tracker.update(measurement=None, dt=0.033)
            assert tracker.status == FilterStatus.COASTING

        state_lost = tracker.update(measurement=None, dt=0.033)
        assert tracker.status == FilterStatus.LOST
        assert state_lost.status == FilterStatus.LOST
        assert state_lost.is_coasting is False
        assert tracker.is_lost is True

    def test_f6_04_seamless_reacquisition_after_occlusion(self):
        """F6.4: Filter re-acquires target after 8 occluded frames and resets coast count."""
        tracker = KalmanPredictiveTracker()
        tracker.initialize(initial_pos=(100.0, 100.0), initial_vel=(100.0, 0.0))

        # Occlude for 8 frames at dt=0.05s (0.4s duration -> delta x = 40px)
        for _ in range(8):
            tracker.update(measurement=None, dt=0.05)
        assert tracker.coast_frames == 8

        # Target emerges at x=140.0
        state = tracker.update(measurement=(140.0, 100.0), dt=0.05)
        assert state.status == FilterStatus.TRACKING
        assert state.coast_frames == 0
        assert not state.is_coasting
        assert abs(state.pos_2d[0] - 140.0) < 2.0

    def test_f6_05_update_from_none_detection_triggers_coasting(self):
        """F6.5: update_from_detection(None) initiates coasting step."""
        tracker = KalmanPredictiveTracker()
        tracker.initialize(initial_pos=(100.0, 100.0), initial_vel=(10.0, 0.0))
        state = tracker.update_from_detection(None, dt=0.033)
        assert state.is_coasting is True
        assert state.coast_frames == 1


class TestTier1FeatureF7SpeedCalculation:
    """F7: Speed Calculation in km/h and 3D Kinematics."""

    def test_f7_01_zero_speed_detection(self):
        """F7.1: Zero velocity target has exactly 0 km/h speed."""
        tracker = KalmanPredictiveTracker()
        tracker.initialize(initial_pos=(320.0, 240.0), initial_vel=(0.0, 0.0), distance=30.0)
        state = tracker.get_state()
        assert state.speed_kmh == pytest.approx(0.0, abs=1e-3)
        assert state.speed_mps == pytest.approx(0.0, abs=1e-3)

    def test_f7_02_pure_horizontal_velocity_speed(self):
        """F7.2: Pure horizontal 100 km/h flyby."""
        tracker = KalmanPredictiveTracker()
        f_px = focal_length_from_hfov(640.0, 70.0)
        dist = 40.0
        vx_mps = 100.0 / 3.6  # 27.778 m/s
        vx_px = (vx_mps * f_px) / dist

        tracker.initialize(initial_pos=(320.0, 240.0), initial_vel=(vx_px, 0.0), distance=dist)
        state = tracker.get_state()
        assert state.speed_kmh == pytest.approx(100.0, rel=1e-2)

    def test_f7_03_pure_vertical_velocity_speed(self):
        """F7.3: Pure vertical 50 km/h climb/descent."""
        tracker = KalmanPredictiveTracker()
        f_px = focal_length_from_hfov(640.0, 70.0)
        dist = 30.0
        vy_mps = 50.0 / 3.6  # 13.889 m/s
        vy_px = (vy_mps * f_px) / dist

        tracker.initialize(initial_pos=(320.0, 240.0), initial_vel=(0.0, -vy_px), distance=dist)
        state = tracker.get_state()
        assert state.speed_kmh == pytest.approx(50.0, rel=1e-2)

    def test_f7_04_3d_combined_velocity_components(self):
        """F7.4: Combined 3D velocity (vX=30 m/s, vY=40 m/s, vZ=0) -> speed=50 m/s = 180 km/h."""
        speed_kmh = velocity_to_speed_kmh(30.0, 40.0, 0.0)
        assert speed_kmh == pytest.approx(180.0, rel=1e-3)

    def test_f7_05_distance_rate_range_velocity(self):
        """F7.5: Distance rate estimation contributes to 3D speed during head-on dive."""
        tracker = KalmanPredictiveTracker()
        # Initialize at 60m
        tracker.update((320.0, 240.0), timestamp=0.0, distance=60.0)
        # Update at 0.1s later with distance 56m -> vZ = -40 m/s
        state = tracker.update((320.0, 240.0), timestamp=0.1, distance=56.0)
        assert state.vel_3d[2] < -10.0  # Approaching turret
        assert state.speed_kmh > 30.0
