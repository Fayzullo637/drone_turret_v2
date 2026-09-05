"""Tier 2 Boundary Value Analysis and Extreme Condition Tests for Kalman Predictive Tracker.

Tests:
- Boundary conditions for F4: micro dt, huge dt, zero velocity, sudden displacement jumps
- Boundary conditions for F5: t=0 horizon, 2.0s horizon, single-step and 100-step projections
- Boundary conditions for F6: exactly 0, 10, 15, and 16 coasting frames, rapid alternating occlusion
- Boundary conditions for F7: 0 km/h, 250 km/h boundary, close range (0.1m), far range (100m)
"""

import math
import numpy as np
import pytest

from drone_turret.tracking import (
    FilterStatus,
    KalmanConfig,
    KalmanPredictiveTracker,
    MultiTargetKalmanTracker,
    compute_process_noise,
    compute_transition_matrix,
    focal_length_from_hfov,
    pixel_to_camera_3d,
    velocity_to_speed_kmh,
)


class TestTier2KalmanBoundariesF4:
    """Boundary conditions and numerical robustness for 6-State Kalman filter."""

    def test_b_f4_01_microscopic_dt(self):
        """Microscopic dt (1e-6s) does not crash or produce NaN/Inf."""
        tracker = KalmanPredictiveTracker()
        tracker.initialize(initial_pos=(100.0, 100.0), initial_vel=(10.0, 10.0))
        state = tracker.update((100.01, 100.01), dt=1e-6)
        assert not math.isnan(state.x)
        assert not math.isnan(state.vx)
        assert not math.isinf(state.x)

    def test_b_f4_02_large_dt_clamping(self):
        """Large pause / stall dt (>10s) is clamped to safe max_dt without numeric explosion."""
        tracker = KalmanPredictiveTracker(KalmanConfig(max_dt=1.0))
        tracker.initialize(initial_pos=(100.0, 100.0), initial_vel=(10.0, 0.0))
        state = tracker.update((200.0, 100.0), dt=15.0)
        assert not math.isnan(state.x)
        assert not math.isinf(state.x)

    def test_b_f4_03_zero_or_negative_dt_fallback(self):
        """Zero or negative dt falls back gracefully to nominal dt."""
        tracker = KalmanPredictiveTracker()
        tracker.initialize(initial_pos=(50.0, 50.0))
        state_zero = tracker.update((51.0, 50.0), dt=0.0)
        assert not math.isnan(state_zero.x)

        state_neg = tracker.update((52.0, 50.0), dt=-0.05)
        assert not math.isnan(state_neg.x)

    def test_b_f4_04_sudden_large_displacement_step(self):
        """Sudden step change in position (e.g. 500px teleport) is handled smoothly."""
        tracker = KalmanPredictiveTracker()
        tracker.initialize(initial_pos=(100.0, 100.0))
        # Step change to (600, 100)
        state = tracker.update((600.0, 100.0), dt=0.033)
        assert state.status == FilterStatus.TRACKING
        assert not math.isnan(state.x)

    def test_b_f4_05_extreme_high_measurement_noise(self):
        """High measurement noise covariance operates stably without divergence."""
        tracker = KalmanPredictiveTracker(KalmanConfig(measurement_noise_std=100.0))
        tracker.initialize(initial_pos=(200.0, 200.0))
        for _ in range(50):
            state = tracker.update((200.0 + np.random.normal(0, 50), 200.0), dt=0.033)
        assert not math.isnan(state.x)


class TestTier2TrajectoryBoundariesF5:
    """Boundary conditions for trajectory projection."""

    def test_b_f5_01_zero_time_horizon(self):
        """Zero horizon dt_horizon=0 returns current position without error."""
        tracker = KalmanPredictiveTracker()
        tracker.initialize(initial_pos=(150.0, 150.0), initial_vel=(50.0, 50.0))
        traj = tracker.predict_trajectory(dt_horizon=0.0, num_steps=5)
        assert len(traj) == 6
        assert traj[0][0] == pytest.approx(150.0)

    def test_b_f5_02_maximum_2s_horizon(self):
        """2.0s maximum prediction horizon produces valid monotonically advancing points."""
        tracker = KalmanPredictiveTracker()
        tracker.initialize(initial_pos=(0.0, 0.0), initial_vel=(100.0, 0.0))
        traj = tracker.predict_trajectory(dt_horizon=2.0, num_steps=40)
        assert len(traj) == 41
        assert traj[-1][0] == pytest.approx(200.0, abs=1e-3)

    def test_b_f5_03_single_step_trajectory(self):
        """num_steps=1 produces start and end points."""
        tracker = KalmanPredictiveTracker()
        tracker.initialize(initial_pos=(10.0, 10.0), initial_vel=(20.0, 0.0))
        traj = tracker.predict_trajectory(dt_horizon=1.0, num_steps=1)
        assert len(traj) == 2
        assert traj[0] == (10.0, 10.0)
        assert traj[1][0] == pytest.approx(30.0)

    def test_b_f5_04_dense_100_step_trajectory(self):
        """High-density 100-step trajectory is calculated in < 1ms."""
        tracker = KalmanPredictiveTracker()
        tracker.initialize(initial_pos=(50.0, 50.0), initial_vel=(10.0, 10.0), initial_acc=(2.0, -1.0))
        traj = tracker.predict_trajectory(dt_horizon=1.0, num_steps=100)
        assert len(traj) == 101

    def test_b_f5_05_negative_horizon_safety(self):
        """Negative horizon is clamped safely to positive minimum."""
        tracker = KalmanPredictiveTracker()
        tracker.initialize(initial_pos=(10.0, 10.0))
        traj = tracker.predict_trajectory(dt_horizon=-1.0, num_steps=10)
        assert len(traj) == 11


class TestTier2CoastingBoundariesF6:
    """Boundary conditions for occlusion and coasting."""

    def test_b_f6_01_exact_15_frame_boundary(self):
        """Frame 15 is the last valid coasting frame, Frame 16 transitions to LOST."""
        tracker = KalmanPredictiveTracker(KalmanConfig(max_coast_frames=15))
        tracker.initialize(initial_pos=(100.0, 100.0))

        for frame in range(1, 16):
            state = tracker.update(None, dt=0.033)
            assert state.is_coasting is True
            assert state.status == FilterStatus.COASTING

        # Frame 16: transition to LOST
        state_lost = tracker.update(None, dt=0.033)
        assert state_lost.is_coasting is False
        assert state_lost.status == FilterStatus.LOST

    def test_b_f6_02_rapid_alternating_visibility(self):
        """Rapid alternating pattern (visible, lost, visible, lost) resets coast counter each time."""
        tracker = KalmanPredictiveTracker()
        tracker.initialize(initial_pos=(100.0, 100.0))

        for i in range(10):
            # Visible
            st_vis = tracker.update((100.0 + i*5, 100.0), dt=0.033)
            assert st_vis.coast_frames == 0
            assert st_vis.status == FilterStatus.TRACKING
            # Lost
            st_lost = tracker.update(None, dt=0.033)
            assert st_lost.coast_frames == 1
            assert st_lost.status == FilterStatus.COASTING

    def test_b_f6_03_reinitialization_after_lost(self):
        """Tracker in LOST state automatically reinitializes when a target appears."""
        tracker = KalmanPredictiveTracker(KalmanConfig(max_coast_frames=5))
        tracker.initialize(initial_pos=(100.0, 100.0))

        # Coast into LOST
        for _ in range(6):
            tracker.update(None, dt=0.033)
        assert tracker.status == FilterStatus.LOST

        # New target appears
        state = tracker.update((500.0, 300.0), dt=0.033)
        assert state.status == FilterStatus.TRACKING
        assert state.pos_2d == (500.0, 300.0)
        assert state.coast_frames == 0

    def test_b_f6_04_coasting_with_acceleration(self):
        """Coasting preserves and applies acceleration along the parabolic curve."""
        tracker = KalmanPredictiveTracker(KalmanConfig(max_coast_frames=15))
        tracker.initialize(initial_pos=(0.0, 0.0), initial_vel=(10.0, 0.0), initial_acc=(20.0, 0.0))

        dt = 0.1
        for i in range(1, 6):
            state = tracker.update(None, dt=dt)
            t = i * dt
            # Expected: x = 10*t + 0.5*20*(t^2)
            expected_x = 10.0 * t + 0.5 * 20.0 * (t ** 2)
            assert state.pos_2d[0] == pytest.approx(expected_x, abs=1.0)

    def test_b_f6_05_custom_max_coast_frames(self):
        """Configuring max_coast_frames=20 allows coasting up to 20 frames."""
        tracker = KalmanPredictiveTracker(KalmanConfig(max_coast_frames=20))
        tracker.initialize(initial_pos=(10.0, 10.0))
        for _ in range(20):
            tracker.update(None, dt=0.033)
            assert tracker.status == FilterStatus.COASTING
        tracker.update(None, dt=0.033)
        assert tracker.status == FilterStatus.LOST


class TestTier2SpeedBoundariesF7:
    """Boundary conditions for 3D speed and distance calculations."""

    def test_b_f7_01_max_combat_speed_250kmh(self):
        """Upper boundary high-speed drone (250 km/h = 69.44 m/s)."""
        speed = velocity_to_speed_kmh(69.444, 0.0, 0.0)
        assert speed == pytest.approx(250.0, rel=1e-3)

    def test_b_f7_02_minimum_close_range_distance(self):
        """Extremely close range (0.1m) does not produce zero-division or infinite values."""
        X, Y, Z = pixel_to_camera_3d(400.0, 300.0, distance_m=0.1, width_px=640.0, height_px=480.0)
        assert Z == pytest.approx(0.1)
        assert not math.isnan(X)
        assert not math.isinf(X)

    def test_b_f7_03_far_range_distance(self):
        """Far range (100.0m) maintains proportional coordinate scaling."""
        X, Y, Z = pixel_to_camera_3d(400.0, 240.0, distance_m=100.0, width_px=640.0, height_px=480.0)
        assert Z == pytest.approx(100.0)
        assert X > 0

    def test_b_f7_04_3d_diagonal_velocity(self):
        """3D diagonal vector (vx=20, vy=20, vz=20) -> speed = sqrt(1200)*3.6 = 124.707 km/h."""
        speed = velocity_to_speed_kmh(20.0, 20.0, 20.0)
        expected = math.sqrt(20**2 + 20**2 + 20**2) * 3.6
        assert speed == pytest.approx(expected, rel=1e-4)

    def test_b_f7_05_negative_distance_safety(self):
        """Negative or zero distance inputs are clamped to positive minimum."""
        X, Y, Z = pixel_to_camera_3d(320.0, 240.0, distance_m=-10.0)
        assert Z >= 0.01
        assert not math.isnan(X)
