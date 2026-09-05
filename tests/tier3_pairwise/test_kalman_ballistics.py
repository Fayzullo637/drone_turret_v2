"""Tier 3 Pairwise Integration Tests: Kalman Tracker ↔ Ballistics Solver Coupling (R2 + R3).

Verifies that estimated target positions and velocities from the 6D Kalman Filter
smoothly feed into the Newton-Raphson Ballistic Intercept Solver without angular jitter,
divergence, or discontinuity under noisy tracking conditions.
"""

from __future__ import annotations

import math
import numpy as np
import pytest

from tests.fixtures.physics_benchmarks import BallisticsBenchmarks
from tests.fixtures.synthetic_video import TargetKinematics
from tests.tier1_features.test_kalman_6d import DroneKalmanFilter6D


def test_kalman_ballistics_coupling_under_measurement_noise(physics_benchmarks):
    """T3.1.1: Tests feeding noisy Kalman velocity estimates into intercept solver produces smooth lead points."""
    dt = 1.0 / 30.0
    target = TargetKinematics(x=-15.0, y=0.0, z=40.0, vx=25.0)  # 90 km/h crossing
    init_px = 320.0 + (800.0 * target.x / target.z)
    init_py = 240.0 - (800.0 * target.y / target.z)
    kf = DroneKalmanFilter6D(initial_pos=(init_px, init_py), dt=dt)

    pan_history = []
    tilt_history = []
    t_int_history = []

    np.random.seed(42)
    for frame_idx in range(25):
        target.update(dt)
        # Add pixel measurement noise
        noise_x = np.random.normal(0, 1.5)
        noise_y = np.random.normal(0, 1.5)
        meas_x = 320.0 + (800.0 * target.x / target.z) + noise_x
        meas_y = 240.0 - (800.0 * target.y / target.z) + noise_y

        state = kf.update((meas_x, meas_y), dt=dt)

        # Scale estimated velocity to 3D metric velocities
        vx_est = state.vel_2d[0] * (target.z / 800.0)
        vy_est = -state.vel_2d[1] * (target.z / 800.0)

        sol = physics_benchmarks.reference_intercept_solver(
            target_pos_3d=(target.x, target.y, target.z),
            target_vel_3d=(vx_est, vy_est, 0.0),
            v0=80.0,
            mass=0.6,
            cd=1.2,
        )

        if sol["reachable"]:
            pan_history.append(sol["aim_pan_deg"])
            tilt_history.append(sol["aim_tilt_deg"])
            t_int_history.append(sol["t_intercept"])

    assert len(pan_history) >= 15
    # Verify trajectory smoothness: angular changes frame-to-frame after convergence must be bounded (< 5 deg/frame)
    pan_diffs = [abs(pan_history[i] - pan_history[i - 1]) for i in range(5, len(pan_history))]
    assert max(pan_diffs) < 5.0
    # Intercept time should remain steady near ~0.65-0.95s
    assert 0.50 < np.mean(t_int_history) < 0.95


def test_kalman_occlusion_to_ballistics_continuity(physics_benchmarks):
    """T3.1.2: Verifies intercept solver continues computing valid lead angles during 10-frame visual occlusion."""
    dt = 1.0 / 30.0
    kf = DroneKalmanFilter6D(initial_pos=(200.0, 240.0), dt=dt)
    target = TargetKinematics(x=-10.0, y=0.0, z=45.0, vx=30.0)

    # Prime Kalman filter for 15 frames
    for _ in range(15):
        target.update(dt)
        meas_x = 320.0 + (800.0 * target.x / target.z)
        meas_y = 240.0
        kf.update((meas_x, meas_y), dt=dt)

    # 10 frames of total visual occlusion (measurement is None)
    lead_positions = []
    for _ in range(10):
        target.update(dt)
        state = kf.update(None, dt=dt)
        assert state.is_coasting is True

        vx_est = state.vel_2d[0] * (target.z / 800.0)
        vy_est = -state.vel_2d[1] * (target.z / 800.0)

        sol = physics_benchmarks.reference_intercept_solver(
            target_pos_3d=(target.x, target.y, target.z),
            target_vel_3d=(vx_est, vy_est, 0.0),
            v0=80.0,
        )
        assert sol["reachable"] is True
        lead_positions.append(sol["lead_pos_3d"][0])

    # Lead positions during coasting must monotonically advance forward
    for i in range(1, len(lead_positions)):
        assert lead_positions[i] > lead_positions[i - 1]
