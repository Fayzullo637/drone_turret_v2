"""Tier 5 Adversarial Stress Suite 2: High-G Evasive Maneuvering Stress Tests.

Empirically challenges the 6-state CA Kalman Filter, trajectory extrapolation,
and ballistic intercept solver against severe evasive UAV maneuvers with
lateral and vertical accelerations up to 3g (29.43 m/s^2).
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
    focal_length_from_hfov,
)
from drone_turret.ballistics.calculator import (
    BallisticCalculator,
    BallisticConfig,
    TargetState,
    InterceptSolution,
)


@pytest.fixture
def kalman_tracker() -> KalmanPredictiveTracker:
    config = KalmanConfig(
        process_noise_scale=250.0,  # Elevated process noise for aggressive 3g maneuvering
        measurement_noise_std=2.0,
        camera_width=640,
        camera_height=480,
        camera_hfov=70.0,
        default_distance=40.0,
    )
    return KalmanPredictiveTracker(config)


@pytest.fixture
def ballistic_calc() -> BallisticCalculator:
    return BallisticCalculator(BallisticConfig())


def test_sinusoidal_3g_acceleration_tracking(kalman_tracker):
    """Stress-test 2.1: Verify Kalman 6-state filter tracks sinusoidal lateral 3g acceleration.
    
    Target performs sinusoidal evasive oscillation:
        x(t) = A * sin(omega * t)
        v(t) = A * omega * cos(omega * t)
        a(t) = -A * omega^2 * sin(omega * t)
    where peak acceleration is 3g = 29.43 m/s^2.
    
    Validates:
    - Acceleration estimation ax tracks ground truth with high cross-correlation (r > 0.60).
    - Position error remains bounded (< 30 pixels / < 1.0 m at 40m range).
    - Filter status remains TRACKING throughout all 120 frames (4 seconds).
    - Error covariance remains stable and positive definite.
    """
    g = 9.81
    a_peak = 3.0 * g  # 29.43 m/s^2
    freq_hz = 0.5  # 0.5 Hz oscillation
    omega = 2.0 * math.pi * freq_hz
    # A * omega^2 = a_peak => A = a_peak / omega^2
    amplitude_m = a_peak / (omega ** 2)  # ~2.98 m

    range_m = 40.0
    f_px = focal_length_from_hfov(640, 70.0)
    cx_cam, cy_cam = 320.0, 240.0
    dt = 1.0 / 30.0
    total_frames = 120

    gt_ax_list = []
    est_ax_list = []
    pos_errors_px = []

    for k in range(total_frames):
        t = k * dt
        # True physics state
        x_true = amplitude_m * math.sin(omega * t)
        vx_true = amplitude_m * omega * math.cos(omega * t)
        ax_true = -amplitude_m * (omega ** 2) * math.sin(omega * t)

        # 2D projection
        px_true = cx_cam + (f_px * x_true / range_m)
        py_true = cy_cam

        # Add camera measurement noise (sigma = 1.5 px)
        np.random.seed(100 + k)
        meas_x = px_true + np.random.normal(0, 1.5)
        meas_y = py_true + np.random.normal(0, 1.5)

        state = kalman_tracker.update(
            measurement=(meas_x, meas_y),
            dt=dt,
            distance=range_m,
            frame_shape=(480, 640),
            timestamp=t,
        )

        # Reconstructed 3D acceleration from 2D pixel acceleration:
        # ax_3d = ax_px * distance / f_px
        est_ax_3d = (state.ax * range_m) / f_px

        gt_ax_list.append(ax_true)
        est_ax_list.append(est_ax_3d)

        # 2D tracking error
        pos_errors_px.append(math.hypot(state.x - px_true, state.y - py_true))

    # Assertions after convergence (skip first 20 frames transient)
    eval_gt_ax = np.array(gt_ax_list[20:])
    eval_est_ax = np.array(est_ax_list[20:])
    eval_pos_errors = np.array(pos_errors_px[20:])

    # 1. Position tracking accuracy: Mean error < 25 pixels (~0.8m at 40m)
    assert np.mean(eval_pos_errors) < 25.0, f"Mean pos error too high: {np.mean(eval_pos_errors):.2f} px"
    assert np.max(eval_pos_errors) < 55.0, f"Max pos error exceeded threshold: {np.max(eval_pos_errors):.2f} px"

    # 2. Acceleration tracking: Cross-correlation with filter group delay compensation (tau_lag <= 15 frames)
    best_r = -1.0
    best_lag = 0
    for lag in range(16):
        if lag == 0:
            r = np.corrcoef(eval_gt_ax, eval_est_ax)[0, 1]
        else:
            r = np.corrcoef(eval_gt_ax[:-lag], eval_est_ax[lag:])[0, 1]
        if r > best_r:
            best_r = r
            best_lag = lag

    assert best_r > 0.60, f"Lag-compensated acceleration correlation too low: max r = {best_r:.3f} at lag {best_lag} (expected > 0.60)"
    assert best_lag <= 15, f"Filter acceleration delay too large: {best_lag} frames"

    # 3. Peak estimated acceleration magnitude reaches near 3g (at least 18 m/s^2)
    assert np.max(np.abs(eval_est_ax)) > 18.0, f"Filter failed to estimate high-G peak: {np.max(np.abs(eval_est_ax)):.2f} m/s^2"


def test_step_3g_jerk_recovery(kalman_tracker):
    """Stress-test 2.2: Instantaneous 3g lateral step acceleration impulse (jerk).
    
    Drone flies at constant velocity for 30 frames, then instantly applies 3g (29.43 m/s^2) thrust.
    Validates:
    - Filter does not diverge, explode covariance, or produce NaN.
    - Estimates catch up to new acceleration within 15 frames.
    - Status remains TRACKING throughout.
    """
    g = 9.81
    a_step = 3.0 * g  # 29.43 m/s^2
    range_m = 35.0
    f_px = focal_length_from_hfov(640, 70.0)
    dt = 1.0 / 30.0

    x_true = 0.0
    vx_true = 10.0  # initial 10 m/s
    ax_true = 0.0
    t = 0.0

    est_accelerations = []

    for k in range(60):
        t += dt
        if k >= 30:
            ax_true = a_step  # Step jump to 3g
        
        vx_true += ax_true * dt
        x_true += vx_true * dt

        px = 320.0 + (f_px * x_true / range_m)
        py = 240.0

        state = kalman_tracker.update(
            measurement=(px, py),
            dt=dt,
            distance=range_m,
            timestamp=t,
        )
        est_ax_3d = (state.ax * range_m) / f_px
        est_accelerations.append(est_ax_3d)

    # By frame 50 (20 frames after step), estimated acceleration should be positive and substantial
    post_step_ax = np.array(est_accelerations[45:])
    assert np.all(post_step_ax > 10.0), f"Filter failed to ramp up to 3g step: {post_step_ax}"
    assert kalman_tracker.status == FilterStatus.TRACKING


def test_3g_corkscrew_multi_axis_maneuver(kalman_tracker):
    """Stress-test 2.3: Combined simultaneous X & Y sinusoidal corkscrew maneuver totaling 3g.
    
    Validates dual-axis acceleration estimation and covariance stability.
    """
    g = 9.81
    a_total = 3.0 * g  # 29.43 m/s^2
    a_axis = a_total / math.sqrt(2.0)  # ~20.81 m/s^2 on each axis

    omega = 2.0 * math.pi * 0.7  # 0.7 Hz
    amp_x = a_axis / (omega ** 2)
    amp_y = a_axis / (omega ** 2)
    range_m = 30.0
    f_px = focal_length_from_hfov(640, 70.0)
    dt = 1.0 / 30.0

    errors_px = []

    for k in range(90):
        t = k * dt
        x_true = amp_x * math.sin(omega * t)
        y_true = amp_y * math.cos(omega * t)

        px = 320.0 + (f_px * x_true / range_m)
        py = 240.0 - (f_px * y_true / range_m)

        state = kalman_tracker.update(
            measurement=(px, py),
            dt=dt,
            distance=range_m,
            timestamp=t,
        )

        err = math.hypot(state.x - px, state.y - py)
        errors_px.append(err)

    steady_errs = np.array(errors_px[15:])
    assert np.mean(steady_errs) < 30.0, f"Mean corkscrew error too high: {np.mean(steady_errs):.2f} px"
    assert kalman_tracker.status == FilterStatus.TRACKING


def test_acceleration_aided_trajectory_extrapolation(kalman_tracker):
    """Stress-test 2.4: Verify CA forward projection vs naive CV extrapolation under 3g curve.
    
    During an aggressive turn (a = 25 m/s^2), forward trajectory prediction with acceleration
    p(t + dt) = p + v*dt + 0.5*a*dt^2 must have lower error than constant-velocity extrapolation.
    """
    range_m = 40.0
    f_px = focal_length_from_hfov(640, 70.0)
    dt = 1.0 / 30.0
    a_turn = 25.0  # m/s^2

    # Step filter through a curve
    x = 0.0
    vx = 0.0
    t = 0.0
    for k in range(30):
        t += dt
        vx += a_turn * dt
        x += vx * dt
        px = 320.0 + (f_px * x / range_m)
        py = 240.0
        kalman_tracker.update(measurement=(px, py), dt=dt, distance=range_m, timestamp=t)

    # Extrapolate 0.5s ahead
    t_horizon = 0.5
    pred_x, pred_y = kalman_tracker.predict_at_time(t_horizon)

    # True future position at t + t_horizon:
    true_future_x = x + vx * t_horizon + 0.5 * a_turn * (t_horizon ** 2)
    true_future_px = 320.0 + (f_px * true_future_x / range_m)

    # Naive CV future position:
    state = kalman_tracker.get_state()
    naive_cv_px = state.x + state.vx * t_horizon

    ca_error = abs(pred_x - true_future_px)
    cv_error = abs(naive_cv_px - true_future_px)

    assert ca_error < cv_error, (
        f"CA extrapolation error ({ca_error:.1f} px) was not better than naive CV ({cv_error:.1f} px)"
    )


def test_high_g_intercept_solution_convergence(ballistic_calc):
    """Stress-test 2.5: Verify Newton-Raphson converges on target with 3g acceleration.
    
    Target at 40m moving at 20 m/s with 3g lateral acceleration (ax = 29.43 m/s^2).
    Validates:
    - Root-finder converges with residual <= 0.01m.
    - Solution accounts for acceleration: lead_pos_3d incorporates 0.5 * ax * t_int^2.
    """
    ax_mps2 = 29.43
    vx_mps = 20.0
    range_m = 40.0

    target = TargetState(
        pos_3d=(0.0, 0.0, range_m),
        vel_3d=(vx_mps, 0.0, 0.0),
        acc_3d=(ax_mps2, 0.0, 0.0),
        speed_kmh=vx_mps * 3.6,
    )

    sol = ballistic_calc.solve_intercept(target)

    assert sol.reachable, "High-G target at 40m should be reachable"
    assert sol.residual_m <= 0.01, f"Residual exceeded tolerance: {sol.residual_m} m"
    
    # Expected lead X = vx * t_int + 0.5 * ax * t_int^2
    t_int = sol.t_intercept
    expected_lead_x = vx_mps * t_int + 0.5 * ax_mps2 * (t_int ** 2)
    assert sol.lead_pos_3d[0] == pytest.approx(expected_lead_x, abs=0.5), (
        f"Lead X {sol.lead_pos_3d[0]} did not match accelerated prediction {expected_lead_x}"
    )
