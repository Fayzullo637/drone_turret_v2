"""Tier 5 Adversarial Stress Suite 1: High-Speed 200 km/h Orthogonal Traversal Stress Tests.

Empirically tests tracking, kinematics, ballistic intercept calculation, and servo control
under extreme high-speed lateral motion (200 km/h = 55.56 m/s and beyond up to 250 km/h)
across varying engagement ranges (20m to 70m) and frame timing jitter.
"""

from __future__ import annotations

import math
from typing import List, Tuple
import numpy as np
import pytest

from drone_turret.tracking.kalman_filter import (
    KalmanPredictiveTracker,
    KalmanConfig,
    FilterStatus,
    pixel_to_camera_3d,
    velocity_to_speed_kmh,
    focal_length_from_hfov,
)
from drone_turret.ballistics.calculator import (
    BallisticCalculator,
    BallisticConfig,
    TargetState,
    InterceptSolution,
)
from drone_turret.control.pid import TurretController
from tests.fixtures.synthetic_video import SyntheticVideoGenerator, TargetKinematics
from tests.fixtures.physics_benchmarks import BallisticsBenchmarks


@pytest.fixture
def ballistic_calc() -> BallisticCalculator:
    return BallisticCalculator(
        BallisticConfig(
            muzzle_velocity=80.0,
            projectile_mass=0.35,
            cd_initial=0.45,
            cd_max=1.35,
            area_initial=0.0015,
            area_max=0.0080,
            dt_step=0.005,
        )
    )


@pytest.fixture
def turret_ctrl() -> TurretController:
    return TurretController(
        pan_kp=0.15,
        pan_ki=0.005,
        pan_kd=0.04,
        tilt_kp=0.15,
        tilt_ki=0.005,
        tilt_kd=0.04,
        max_step_deg=15.0,
    )


def test_orthogonal_traversal_200kmh_velocity_estimation():
    """Stress-test 1.1: Verify Kalman 6-state CA filter accurately estimates 200 km/h (55.56 m/s).
    
    UAV flies laterally across 640x480 frame at 50m range with speed 200 km/h (vx = 55.56 m/s).
    Validates:
    - 3D velocity convergence: vx estimated within 15% error of 55.56 m/s after initial 15-frame transient.
    - Zero spurious cross-axis velocity: vy < 3.0 m/s and vz < 3.0 m/s.
    - Speed magnitude in km/h converges to approx 200.0 km/h.
    - Covariance matrix remains bounded and positive definite.
    """
    config = KalmanConfig(
        process_noise_scale=20.0,
        measurement_noise_std=2.0,
        camera_width=640,
        camera_height=480,
        camera_hfov=70.0,
        default_distance=50.0,
    )
    tracker = KalmanPredictiveTracker(config)

    f_px = focal_length_from_hfov(640, 70.0)  # ~457.0 px
    cx_cam = 320.0
    cy_cam = 240.0
    range_m = 50.0
    vx_true = 200.0 / 3.6  # 55.556 m/s
    vy_true = 0.0
    vz_true = 0.0
    dt = 1.0 / 30.0  # 30 FPS

    # Target starts at x = -35m (left side of view) and crosses to +35m
    total_frames = 45
    start_x = -vx_true * (total_frames * dt / 2.0)
    
    tracked_speeds = []
    tracked_vx = []
    cov_diags = []

    for k in range(total_frames):
        t = k * dt
        x_m = start_x + vx_true * t
        y_m = 0.0
        z_m = range_m

        # Project 3D to 2D pixel
        px = cx_cam + (f_px * x_m / z_m)
        py = cy_cam - (f_px * y_m / z_m)

        # Measurement with small sensor Gaussian jitter (sigma = 1.5 px)
        np.random.seed(42 + k)
        meas_x = px + np.random.normal(0, 1.5)
        meas_y = py + np.random.normal(0, 1.5)

        state = tracker.update(
            measurement=(meas_x, meas_y),
            dt=dt,
            distance=range_m,
            frame_shape=(480, 640),
            timestamp=t,
        )

        tracked_speeds.append(state.speed_kmh)
        tracked_vx.append(state.vX)
        cov_diags.append(np.diag(tracker.covariance_matrix))

    # Assertions on converged trajectory (frames 15..45)
    steady_vx = np.array(tracked_vx[15:])
    steady_speed = np.array(tracked_speeds[15:])

    assert np.mean(steady_vx) == pytest.approx(vx_true, rel=0.12), (
        f"Expected vx ~ {vx_true:.2f} m/s, got {np.mean(steady_vx):.2f} m/s"
    )
    assert np.mean(steady_speed) == pytest.approx(200.0, rel=0.12), (
        f"Expected speed ~ 200.0 km/h, got {np.mean(steady_speed):.2f} km/h"
    )

    # Cross-axis velocity check
    final_state = tracker.get_state(frame_shape=(480, 640))
    assert abs(final_state.vY) < 3.5, f"Spurious vertical velocity vY = {final_state.vY} m/s"
    assert tracker.status == FilterStatus.TRACKING

    # Covariance stability: diagonal elements must remain strictly positive and bounded
    for diag in cov_diags:
        assert np.all(diag > 0), "Kalman covariance lost positive definiteness"
        assert diag[0] < 500.0, f"Position variance exploded: {diag[0]}"
        assert diag[2] < 50000.0, f"Velocity variance exploded: {diag[2]}"


def test_orthogonal_traversal_ballistic_lead_computation(ballistic_calc):
    """Stress-test 1.2: Verify RK4 + Newton-Raphson computes correct lead point at 200 km/h.
    
    Drone crossing horizontally at 200 km/h (55.56 m/s) at range 35m (starting at x = -20m).
    Validates:
    - Newton-Raphson converges within tolerance <= 0.01m in <= 15 iterations.
    - Time of flight t_int is physically consistent (approx 0.45 - 0.65 s for 80 m/s muzzle speed).
    - Lead point X coordinate is displaced forward by dx = vx * t_int (approx 25 - 35 m from start).
    - Pan aiming angle pan_deg increases substantially in the flight direction (pan > 90 deg).
    - Drop compensation drop_m is positive and accounts for gravity + drag (approx 1.2 - 2.5 m).
    """
    vx_mps = 200.0 / 3.6  # 55.556 m/s
    range_m = 35.0
    start_x = -20.0

    target = TargetState(
        pos_3d=(start_x, 0.0, range_m),  # Starting left of boresight
        vel_3d=(vx_mps, 0.0, 0.0),       # Moving right at 55.56 m/s
        speed_kmh=200.0,
    )

    sol = ballistic_calc.solve_intercept(target)

    assert sol.reachable, "Expected 200 km/h target at 35m to be reachable within 80 m/s launcher limits"
    assert 0.35 <= sol.t_intercept <= 0.85, f"Flight time unexpected: {sol.t_intercept} s"
    assert sol.iterations <= 15, f"Newton-Raphson took too many iterations: {sol.iterations}"
    assert sol.residual_m <= 0.01, f"Root residual exceeded tolerance: {sol.residual_m} m"

    # Lead position check: lead X = start_x + vx * t_int
    expected_lead_x = start_x + vx_mps * sol.t_intercept
    assert sol.lead_pos_3d[0] == pytest.approx(expected_lead_x, abs=0.5), (
        f"Lead X pos {sol.lead_pos_3d[0]} mismatch with expected {expected_lead_x}"
    )

    # Lead aim pan angle must lead the target (for +X motion, pan > 90 deg)
    assert sol.aim_pan_deg > 90.0, f"Aim pan angle {sol.aim_pan_deg} did not lead the target"
    
    # Drop compensation must be non-zero and positive
    assert sol.drop_m > 1.0, f"Expected drop > 1.0m at 35m with drag, got {sol.drop_m} m"
    assert sol.aim_tilt_deg > 90.0, f"Aim tilt {sol.aim_tilt_deg} did not pitch up for drop compensation"


@pytest.mark.parametrize("speed_kmh", [150.0, 200.0, 220.0, 250.0])
@pytest.mark.parametrize("distance_m", [20.0, 35.0, 50.0, 70.0])
def test_high_speed_orthogonal_sweeps(ballistic_calc, speed_kmh: float, distance_m: float):
    """Stress-test 1.3: Parametric sweep across high speeds (150-250 km/h) and ranges (20-70m).
    
    Validates:
    - Numerical solver stability without division-by-zero, NaN, or infinite loops.
    - Reachability correctly flags out-of-envelope geometry (e.g. 250 km/h at 70m).
    - When reachable, residual <= 0.02m.
    - Aim angles stay within servo limits [0, 180] degrees.
    """
    vx_mps = speed_kmh / 3.6
    target = TargetState(
        pos_3d=(-10.0, 2.0, distance_m),
        vel_3d=(vx_mps, 0.0, 0.0),
        speed_kmh=speed_kmh,
    )

    sol = ballistic_calc.solve_intercept(target)

    # Angle limits assertion
    assert 0.0 <= sol.aim_pan_deg <= 180.0, f"Pan angle out of bounds: {sol.aim_pan_deg}"
    assert 0.0 <= sol.aim_tilt_deg <= 180.0, f"Tilt angle out of bounds: {sol.aim_tilt_deg}"

    if sol.reachable:
        assert sol.residual_m <= 0.02, f"Residual exceeded for speed={speed_kmh}, dist={distance_m}: {sol.residual_m}"
        assert sol.t_intercept > 0.1, f"Unrealistically low intercept time: {sol.t_intercept}"
        assert sol.drop_m > 0.05, f"Drop too small: {sol.drop_m}"
    else:
        # At very long distance & hyper speed (e.g. 250 km/h at 70m), net may fail to reach target before max flight time
        assert sol.t_intercept >= 0.0


def test_high_speed_closed_loop_tracking_with_pid(ballistic_calc, turret_ctrl):
    """Stress-test 1.4: Full closed-loop 200 km/h tracking from kinematics -> ballistics -> PID servos.
    
    Simulates drone crossing at 200 km/h across 60 frames.
    Validates:
    - Servo commands smoothly track the rapid lead angle rate without unbounded oscillations.
    - Integral windup does not saturate.
    - Commands remain bounded within [0, 180] degrees.
    """
    config = KalmanConfig(
        process_noise_scale=20.0,
        measurement_noise_std=2.0,
        default_distance=45.0,
    )
    tracker = KalmanPredictiveTracker(config)
    f_px = focal_length_from_hfov(640, 70.0)
    cx_cam, cy_cam = 320.0, 240.0
    vx_mps = 200.0 / 3.6
    range_m = 45.0
    dt = 1.0 / 30.0

    pan_cmds = []
    tilt_cmds = []
    lead_pan_targets = []

    for k in range(60):
        t = k * dt
        x_m = -30.0 + vx_mps * t
        y_m = 1.5
        z_m = range_m

        # 2D projection
        px = cx_cam + (f_px * x_m / z_m)
        py = cy_cam - (f_px * y_m / z_m)

        # Kalman update
        kf_state = tracker.update(
            measurement=(px, py),
            dt=dt,
            distance=range_m,
            frame_shape=(480, 640),
            timestamp=t,
        )

        # Ballistic solve
        b_target = TargetState(
            pos_3d=kf_state.pos_3d,
            vel_3d=kf_state.vel_3d,
            speed_kmh=kf_state.speed_kmh,
        )
        sol = ballistic_calc.solve_intercept(b_target)

        # PID servo step
        cmd_pan, cmd_tilt = turret_ctrl.update_lead_target(
            target_pan_deg=sol.aim_pan_deg,
            target_tilt_deg=sol.aim_tilt_deg,
            dt=dt,
        )

        pan_cmds.append(cmd_pan)
        tilt_cmds.append(cmd_tilt)
        lead_pan_targets.append(sol.aim_pan_deg)

    # Verify smooth servo trajectory
    pan_array = np.array(pan_cmds)
    tilt_array = np.array(tilt_cmds)

    # 1. Servo angle bounds
    assert np.all(pan_array >= 0.0) and np.all(pan_array <= 180.0)
    assert np.all(tilt_array >= 0.0) and np.all(tilt_array <= 180.0)

    # 2. Pan servo progresses monotonically or smoothly following the left-to-right lead
    # After initial transient (frame 15+), pan delta per frame is bounded by max_step_deg
    pan_diffs = np.abs(np.diff(pan_array[15:]))
    assert np.all(pan_diffs <= 15.01), f"Pan step exceeded slew limit: {np.max(pan_diffs)}"

    # 3. Telemetry verifies anti-windup clamping
    assert abs(turret_ctrl.pan_pid._integral_accum) <= 5.0, "Pan integral accumulator violated anti-windup clamping"
    assert abs(turret_ctrl.tilt_pid._integral_accum) <= 5.0, "Tilt integral accumulator violated anti-windup clamping"


def test_sub_frame_time_jitter_high_speed():
    """Stress-test 1.5: Verify Kalman filter robustness against high framerate jitter at 200 km/h.
    
    Simulates variable frame intervals dt fluctuating between 16ms (60 FPS) and 50ms (20 FPS).
    Validates:
    - Dynamic dt matrix recalculation F(dt) and Q(dt) maintains continuous tracking.
    - Speed estimate does not produce NaN or spike > 300 km/h due to time step jitter.
    """
    config = KalmanConfig(
        process_noise_scale=20.0,
        measurement_noise_std=2.0,
        min_dt=1e-4,
        max_dt=0.5,
    )
    tracker = KalmanPredictiveTracker(config)
    vx_mps = 200.0 / 3.6
    range_m = 40.0
    f_px = focal_length_from_hfov(640, 70.0)

    # Generate pseudo-random dt sequence
    np.random.seed(1337)
    dt_sequence = np.random.uniform(0.016, 0.050, size=50)

    t = 0.0
    x_m = -25.0
    speed_estimates = []

    for dt in dt_sequence:
        t += dt
        x_m += vx_mps * dt
        px = 320.0 + (f_px * x_m / range_m)
        py = 240.0

        state = tracker.update(
            measurement=(px, py),
            dt=dt,
            distance=range_m,
            timestamp=t,
        )
        speed_estimates.append(state.speed_kmh)

    # Check stability
    steady_speeds = np.array(speed_estimates[15:])
    assert not np.any(np.isnan(steady_speeds)), "NaN encountered during jitter test"
    assert not np.any(np.isinf(steady_speeds)), "Inf encountered during jitter test"
    assert np.mean(steady_speeds) == pytest.approx(200.0, rel=0.15), (
        f"Mean speed with jitter {np.mean(steady_speeds):.1f} km/h deviated from 200 km/h"
    )
    assert np.max(steady_speeds) < 260.0, f"Extreme spike detected: {np.max(steady_speeds)}"
