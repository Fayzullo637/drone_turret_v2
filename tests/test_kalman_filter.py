"""Comprehensive Unit and Physics Verification Tests for 6-State Kalman Predictive Tracker.

Tests:
- Constant Acceleration (CA) 6-state Kalman filter dynamics (F4)
- Dynamic dt transition and process noise adaptation (F4)
- Multi-horizon forward trajectory projection (F5)
- Occlusion & detection loss coasting >= 10-15 frames (F6)
- 3D velocity and speed in km/h (F7)
- MultiTargetKalmanTracker ensemble management
"""

import math
import numpy as np
import pytest

from drone_turret.tracking import (
    FilterConfig,
    FilterStatus,
    KalmanConfig,
    KalmanPredictiveTracker,
    MultiTargetKalmanTracker,
    TargetState,
    compute_process_noise,
    compute_transition_matrix,
    focal_length_from_hfov,
    pixel_to_camera_3d,
    velocity_to_speed_kmh,
)


class TestKalmanMatricesAndMath:
    """Mathematical verification of matrix constructors and kinematics."""

    def test_transition_matrix_structure(self):
        """Verify F(dt) matches analytical 6-state CA transition equations."""
        dt = 0.05
        F = compute_transition_matrix(dt)
        assert F.shape == (6, 6)
        assert F.dtype == np.float32

        # State: [x, y, vx, vy, ax, ay]
        # x(t+dt) = x + vx*dt + 0.5*ax*dt^2
        assert F[0, 0] == pytest.approx(1.0)
        assert F[0, 2] == pytest.approx(dt)
        assert F[0, 4] == pytest.approx(0.5 * dt * dt)

        # y(t+dt) = y + vy*dt + 0.5*ay*dt^2
        assert F[1, 1] == pytest.approx(1.0)
        assert F[1, 3] == pytest.approx(dt)
        assert F[1, 5] == pytest.approx(0.5 * dt * dt)

        # vx(t+dt) = vx + ax*dt
        assert F[2, 2] == pytest.approx(1.0)
        assert F[2, 4] == pytest.approx(dt)

        # vy(t+dt) = vy + ay*dt
        assert F[3, 3] == pytest.approx(1.0)
        assert F[3, 5] == pytest.approx(dt)

        # ax, ay constant
        assert F[4, 4] == pytest.approx(1.0)
        assert F[5, 5] == pytest.approx(1.0)

    def test_process_noise_symmetry_and_positive_definiteness(self):
        """Verify Q(dt) continuous white noise acceleration matrix is symmetric positive semi-definite."""
        dt = 0.0333
        q = 10.0
        Q = compute_process_noise(dt, q=q)
        assert Q.shape == (6, 6)
        assert Q.dtype == np.float32

        # Symmetry test: Q == Q^T
        np.testing.assert_allclose(Q, Q.T, rtol=1e-5, atol=1e-7)

        # Positive semi-definiteness: all eigenvalues >= 0
        eigenvals = np.linalg.eigvals(Q)
        assert np.all(eigenvals >= -1e-7)

    def test_focal_length_calculation(self):
        """Verify focal length calculation for standard webcams."""
        # 640x480 with 70 deg HFOV
        f_640 = focal_length_from_hfov(640.0, 70.0)
        # f = 640 / (2 * tan(35 deg)) = 640 / (2 * 0.7002075) = 457.007
        assert f_640 == pytest.approx(457.007, rel=1e-3)

        # 1920x1080 with 70 deg HFOV
        f_1080 = focal_length_from_hfov(1920.0, 70.0)
        assert f_1080 == pytest.approx(1371.02, rel=1e-3)

    def test_pixel_to_camera_3d_projection(self):
        """Verify 3D coordinate recovery from pixel and distance."""
        # Image center -> should map to X=0, Y=0, Z=distance
        cx, cy = 320.0, 240.0
        dist = 25.0
        X, Y, Z = pixel_to_camera_3d(cx, cy, dist, width_px=640.0, height_px=480.0)
        assert X == pytest.approx(0.0, abs=1e-4)
        assert Y == pytest.approx(0.0, abs=1e-4)
        assert Z == pytest.approx(25.0, abs=1e-4)

        # Point to the right and top
        X_r, Y_t, Z_r = pixel_to_camera_3d(420.0, 140.0, dist, width_px=640.0, height_px=480.0)
        assert X_r > 0  # +X is right
        assert Y_t > 0  # +Y is up (140 is above center 240)
        assert Z_r == pytest.approx(25.0)

    def test_velocity_to_speed_kmh(self):
        """Verify speed conversions: 27.778 m/s = 100.0 km/h, 55.556 m/s = 200.0 km/h."""
        speed_100 = velocity_to_speed_kmh(27.7778, 0.0, 0.0)
        assert speed_100 == pytest.approx(100.0, rel=1e-3)

        speed_200 = velocity_to_speed_kmh(0.0, -55.5556, 0.0)
        assert speed_200 == pytest.approx(200.0, rel=1e-3)

        # 3D vector: vx=30, vy=40, vz=0 -> v_mag=50 m/s = 180 km/h
        speed_3d = velocity_to_speed_kmh(30.0, 40.0, 0.0)
        assert speed_3d == pytest.approx(180.0, rel=1e-3)


class TestKalmanPredictiveTrackerFeatureF4:
    """F4: 6-State Kalman Filter dynamics and dynamic dt updates."""

    def test_initialization_and_reset(self):
        """Tracker starts uninitialized, initializes on first detection, and cleans on reset."""
        tracker = KalmanPredictiveTracker()
        assert tracker.status == FilterStatus.UNINITIALIZED
        assert tracker.is_lost
        assert not tracker.is_tracking

        # First measurement initializes tracker
        state = tracker.update((100.0, 200.0), dt=0.033, distance=30.0)
        assert tracker.status == FilterStatus.TRACKING
        assert tracker.is_tracking
        assert state.pos_2d == (100.0, 200.0)
        assert state.coast_frames == 0
        assert not state.is_coasting

        # Reset returns to uninitialized
        tracker.reset()
        assert tracker.status == FilterStatus.UNINITIALIZED
        assert tracker.is_lost

    def test_constant_velocity_tracking_convergence(self):
        """Filter tracks a drone moving at constant velocity vx=150 px/s, vy=-50 px/s."""
        tracker = KalmanPredictiveTracker()
        dt = 1.0 / 30.0  # 30 FPS
        true_vx = 150.0
        true_vy = -50.0
        cur_x, cur_y = 100.0, 300.0

        # Simulate 60 frames (2 seconds)
        for i in range(60):
            cur_x += true_vx * dt
            cur_y += true_vy * dt
            # Add slight sensor noise (+-1 px)
            noise_x = 0.5 * (1 if i % 2 == 0 else -1)
            noise_y = 0.5 * (-1 if i % 2 == 0 else 1)
            state = tracker.update((cur_x + noise_x, cur_y + noise_y), dt=dt, distance=40.0)

        # Velocity estimates should converge close to ground truth
        assert state.pos_2d[0] == pytest.approx(cur_x, abs=3.0)
        assert state.pos_2d[1] == pytest.approx(cur_y, abs=3.0)
        assert state.vel_2d[0] == pytest.approx(true_vx, abs=15.0)
        assert state.vel_2d[1] == pytest.approx(true_vy, abs=15.0)

    def test_constant_acceleration_tracking(self):
        """Filter tracks an accelerating drone with ax=50 px/s^2."""
        tracker = KalmanPredictiveTracker(KalmanConfig(process_noise_scale=20.0))
        dt = 0.0333
        true_ax = 50.0
        cur_vx = 0.0
        cur_x = 50.0

        for i in range(90):  # 3 seconds
            cur_vx += true_ax * dt
            cur_x += cur_vx * dt
            state = tracker.update((cur_x, 200.0), dt=dt, distance=30.0)

        # Acceleration estimate should be positive and close to 50
        assert state.acc_2d[0] == pytest.approx(true_ax, abs=20.0)
        assert state.vel_2d[0] == pytest.approx(cur_vx, abs=20.0)

    def test_dynamic_dt_fluctuations(self):
        """Tracker handles varying frame intervals (jittering between 10ms and 100ms)."""
        tracker = KalmanPredictiveTracker()
        dts = [0.016, 0.033, 0.050, 0.020, 0.080, 0.033, 0.010]
        cur_x = 100.0
        vx = 200.0

        for dt in dts * 5:  # 35 updates with varying dt
            cur_x += vx * dt
            state = tracker.update((cur_x, 150.0), dt=dt)
            assert not math.isnan(state.x)
            assert not math.isnan(state.vx)

        assert state.vel_2d[0] == pytest.approx(vx, abs=25.0)


class TestKalmanTrajectoryProjectionFeatureF5:
    """F5: Forward multi-horizon trajectory projection."""

    def test_trajectory_projection_points_and_horizon(self):
        """Verify predict_trajectory generates exact number of steps and covers horizon."""
        tracker = KalmanPredictiveTracker()
        tracker.initialize(
            initial_pos=(200.0, 150.0),
            initial_vel=(100.0, 50.0),
            initial_acc=(10.0, -5.0),
        )

        horizon = 1.0
        num_steps = 20
        traj = tracker.predict_trajectory(dt_horizon=horizon, num_steps=num_steps)

        assert len(traj) == num_steps + 1  # t=0 to t=1.0 inclusive
        # First point is current position
        assert traj[0][0] == pytest.approx(200.0)
        assert traj[0][1] == pytest.approx(150.0)

        # Final point at t=1.0s:
        # x(1.0) = 200 + 100*(1.0) + 0.5*10*(1.0)^2 = 305.0
        # y(1.0) = 150 + 50*(1.0) + 0.5*(-5)*(1.0)^2 = 197.5
        assert traj[-1][0] == pytest.approx(305.0)
        assert traj[-1][1] == pytest.approx(197.5)

    def test_multi_horizon_range(self):
        """Verify predictions across various horizons [0.1s, 0.5s, 1.0s, 2.0s]."""
        tracker = KalmanPredictiveTracker()
        tracker.initialize(initial_pos=(0.0, 0.0), initial_vel=(50.0, 0.0))

        for horizon in [0.1, 0.5, 1.0, 2.0]:
            traj = tracker.predict_trajectory(dt_horizon=horizon, num_steps=10)
            assert len(traj) == 11
            expected_x_end = 50.0 * horizon
            assert traj[-1][0] == pytest.approx(expected_x_end, abs=1e-3)

    def test_3d_trajectory_projection(self):
        """Verify predict_trajectory_3d outputs valid 3D points."""
        tracker = KalmanPredictiveTracker()
        tracker.initialize(
            initial_pos=(320.0, 240.0),
            initial_vel=(0.0, 0.0),
            distance=50.0,
        )
        traj_3d = tracker.predict_trajectory_3d(dt_horizon=1.0, num_steps=10)
        assert len(traj_3d) == 11
        # At image center, X=0, Y=0, Z=50
        assert traj_3d[0][0] == pytest.approx(0.0, abs=1e-3)
        assert traj_3d[0][1] == pytest.approx(0.0, abs=1e-3)
        assert traj_3d[0][2] == pytest.approx(50.0, abs=1e-3)


class TestKalmanCoastingFeatureF6:
    """F6: Target Loss & Occlusion Coasting (>= 10-15 frames)."""

    def test_coasting_maintains_prediction_during_occlusion(self):
        """When measurement is None, filter coasts forward along trajectory."""
        tracker = KalmanPredictiveTracker(KalmanConfig(max_coast_frames=15))
        tracker.initialize(initial_pos=(100.0, 200.0), initial_vel=(60.0, 0.0))

        dt = 0.1
        # Coast for 10 frames
        for i in range(1, 11):
            state = tracker.update(measurement=None, dt=dt)
            assert state.is_coasting
            assert tracker.status == FilterStatus.COASTING
            assert state.coast_frames == i
            # Position should continue advancing: x = 100 + 60*(i*0.1)
            expected_x = 100.0 + 60.0 * (i * dt)
            assert state.pos_2d[0] == pytest.approx(expected_x, abs=1.0)

    def test_transition_to_lost_after_max_coast_frames(self):
        """Filter transitions to LOST after max_coast_frames (15 frames)."""
        tracker = KalmanPredictiveTracker(KalmanConfig(max_coast_frames=15))
        tracker.initialize(initial_pos=(100.0, 100.0), initial_vel=(10.0, 0.0))

        for frame in range(1, 16):
            state = tracker.update(measurement=None, dt=0.033)
            assert tracker.status == FilterStatus.COASTING
            assert state.coast_frames == frame

        # 16th frame exceeds max_coast_frames (15) -> transitions to LOST
        state_lost = tracker.update(measurement=None, dt=0.033)
        assert tracker.status == FilterStatus.LOST
        assert tracker.is_lost
        assert not tracker.is_tracking

    def test_recovery_from_coasting_on_redetection(self):
        """Filter smoothly resumes TRACKING when measurement returns during coasting."""
        tracker = KalmanPredictiveTracker(KalmanConfig(max_coast_frames=15))
        tracker.initialize(initial_pos=(100.0, 100.0), initial_vel=(30.0, 0.0))

        # Coast for 8 frames
        for _ in range(8):
            tracker.update(measurement=None, dt=0.1)
        assert tracker.status == FilterStatus.COASTING
        assert tracker.coast_frames == 8

        # Target reappears at x=340.0
        state = tracker.update(measurement=(340.0, 100.0), dt=0.1)
        assert tracker.status == FilterStatus.TRACKING
        assert tracker.coast_frames == 0
        assert not state.is_coasting
        assert state.pos_2d[0] == pytest.approx(340.0, abs=5.0)

    def test_uncertainty_growth_during_coasting(self):
        """Position uncertainty increases monotonically during coasting as time progresses."""
        tracker = KalmanPredictiveTracker()
        tracker.initialize(initial_pos=(100.0, 100.0), initial_vel=(20.0, 0.0))
        initial_unc = tracker.get_state().pos_uncertainty_px

        prev_unc = initial_unc
        for _ in range(10):
            state = tracker.update(measurement=None, dt=0.05)
            assert state.pos_uncertainty_px >= prev_unc
            prev_unc = state.pos_uncertainty_px


class TestKalmanSpeedCalculationFeatureF7:
    """F7: Real-world speed calculation in km/h from pixel velocity and distance."""

    def test_stationary_drone_speed(self):
        """Stationary drone has ~0 km/h speed."""
        tracker = KalmanPredictiveTracker()
        tracker.initialize(initial_pos=(320.0, 240.0), initial_vel=(0.0, 0.0), distance=30.0)
        state = tracker.get_state()
        assert state.speed_kmh == pytest.approx(0.0, abs=1e-2)

    def test_high_speed_flyby_200kmh(self):
        """Orthogonal 200 km/h flyby across camera FOV at 50m range."""
        tracker = KalmanPredictiveTracker()
        # 200 km/h = 55.556 m/s
        # At 50m distance with f = 457.0 px (70 deg HFOV, 640w):
        # vx_px = (vx_m * f) / distance = (55.556 * 457.007) / 50.0 = 507.785 px/s
        distance = 50.0
        f_px = focal_length_from_hfov(640.0, 70.0)
        vx_mps = 200.0 / 3.6  # 55.556 m/s
        vx_px_s = (vx_mps * f_px) / distance

        tracker.initialize(
            initial_pos=(320.0, 240.0),
            initial_vel=(vx_px_s, 0.0),
            distance=distance,
        )
        state = tracker.get_state()
        assert state.vel_3d[0] == pytest.approx(vx_mps, rel=1e-3)
        assert state.speed_kmh == pytest.approx(200.0, rel=1e-2)

    def test_passive_optical_distance_estimation_speed(self):
        """When distance is not supplied, optical bbox ranging estimates distance and speed."""
        tracker = KalmanPredictiveTracker(KalmanConfig(known_drone_size=0.35))
        # Bbox width = 35 px at 640x480 (f ~ 457 px) -> dist = (0.35 * 457) / 35 = 4.57m
        # Centroid at (200, 200)
        bbox = (182, 182, 217, 217)  # w=35, h=35
        tracker.initialize(initial_pos=(200.0, 200.0), initial_vel=(100.0, 0.0), bbox=bbox)
        
        state = tracker.update(measurement=(210.0, 200.0), dt=0.1, bbox=bbox)
        assert state.pos_3d[2] == pytest.approx(4.57, rel=0.1)
        assert state.speed_kmh > 0.0


class TestMultiTargetKalmanTracker:
    """Multi-target tracker ensemble management."""

    def test_multiple_target_tracks_and_locking(self):
        """Track multiple simultaneous targets and switch locked target."""
        multi_tracker = MultiTargetKalmanTracker()

        class MockDetection:
            def __init__(self, track_id, bbox):
                self.track_id = track_id
                self.box = bbox
                self.confidence = 0.9

        det1 = MockDetection(1, (100, 100, 140, 140))
        det2 = MockDetection(2, (400, 200, 450, 250))

        states = multi_tracker.update_tracks([det1, det2], dt=0.033)
        assert len(states) == 2
        assert 1 in states and 2 in states
        assert states[1].pos_2d == (120.0, 120.0)
        assert states[2].pos_2d == (425.0, 225.0)

        # Lock onto target 2
        multi_tracker.lock_track(2)
        locked = multi_tracker.get_locked_target_state()
        assert locked is not None
        assert locked.track_id == 2
        assert locked.pos_2d == (425.0, 225.0)

    def test_multi_target_pruning(self):
        """Lost tracks are automatically pruned while active tracks remain."""
        multi_tracker = MultiTargetKalmanTracker(KalmanConfig(max_coast_frames=5))

        class MockDetection:
            def __init__(self, track_id, bbox):
                self.track_id = track_id
                self.box = bbox
                self.confidence = 0.9

        # Frame 1: both visible
        multi_tracker.update_tracks([
            MockDetection(1, (100, 100, 140, 140)),
            MockDetection(2, (200, 200, 240, 240)),
        ], dt=0.033)

        # Frames 2..10: target 2 disappears
        for _ in range(8):
            states = multi_tracker.update_tracks([
                MockDetection(1, (105, 100, 145, 140)),
            ], dt=0.033)

        # Target 2 should be pruned after 5 coast frames
        assert 1 in multi_tracker.trackers
        assert 2 not in multi_tracker.trackers
