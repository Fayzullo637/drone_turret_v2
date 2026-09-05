"""Tier 1 Unit Tests: BallisticCalculator (Features F8, F9, F10, F11).

Tests the production implementation in drone_turret.ballistics:
- RK4 numerical integration in 3D and 1D
- Dynamic net expansion model Cd(t) and A(t)
- Newton-Raphson intercept solver
- Lead aim angle calculation (Pan/Tilt) with gravity drop compensation
- Tactical HUD 2D image pixel projection
"""

import math
import numpy as np
import pytest

from drone_turret.ballistics import (
    BallisticCalculator,
    BallisticConfig,
    InterceptSolution,
    TargetState,
    TrajectoryPoint,
)


def test_ballistics_config_defaults_and_mutation():
    """F8/F9.1: Verifies default config values and runtime parameter modification."""
    calc = BallisticCalculator()
    assert calc.config.muzzle_velocity == 80.0
    assert calc.config.projectile_mass == 0.35
    assert calc.config.cd_initial == 0.45
    assert calc.config.cd_max == 1.35
    assert calc.config.gravity == 9.81
    assert calc.config.dynamic_expansion is True

    # Runtime update
    calc.update_config(muzzle_velocity=60.0, cd_max=1.50)
    assert calc.config.muzzle_velocity == 60.0
    assert calc.config.cd_max == 1.50

    with pytest.raises(AttributeError):
        calc.update_config(invalid_param_name=123)


def test_dynamic_net_expansion_aerodynamics():
    """F9.1: Verifies Cd(t) and A(t) expand exponentially from canister to deployed mesh."""
    calc = BallisticCalculator()

    # Initial time t=0 (canister dimensions)
    assert calc.cd_at_time(0.0) == pytest.approx(0.45, abs=1e-5)
    assert calc.area_at_time(0.0) == pytest.approx(0.0015, abs=1e-5)

    # Deployment time t = tau (63.2% deployed)
    tau = calc.config.tau_deploy
    expected_cd_tau = 0.45 + (1.35 - 0.45) * (1.0 - math.exp(-1.0))
    expected_area_tau = 0.0015 + (0.0080 - 0.0015) * (1.0 - math.exp(-1.0))
    assert calc.cd_at_time(tau) == pytest.approx(expected_cd_tau, abs=1e-5)
    assert calc.area_at_time(tau) == pytest.approx(expected_area_tau, abs=1e-5)

    # Full deployment at t >> tau
    assert calc.cd_at_time(2.0) == pytest.approx(1.35, abs=1e-3)
    assert calc.area_at_time(2.0) == pytest.approx(0.0080, abs=1e-4)

    # Monotonic expansion check
    times = [0.0, 0.05, 0.10, 0.15, 0.30, 0.50, 1.0]
    cds = [calc.cd_at_time(t) for t in times]
    areas = [calc.area_at_time(t) for t in times]
    for i in range(1, len(times)):
        assert cds[i] >= cds[i - 1]
        assert areas[i] >= areas[i - 1]


def test_drag_acceleration_vector_direction():
    """F9.2: Verifies drag deceleration opposes velocity vector and scales with speed."""
    calc = BallisticCalculator()
    v_fwd = np.array([0.0, 0.0, 80.0])
    a_drag = calc.drag_acceleration(0.5, v_fwd)

    # Acceleration must point in -Z direction
    assert a_drag[0] == 0.0
    assert a_drag[1] == 0.0
    assert a_drag[2] < 0.0

    # Zero velocity produces zero drag
    a_zero = calc.drag_acceleration(0.5, np.array([0.0, 0.0, 0.0]))
    assert np.all(a_zero == 0.0)


def test_rk4_vacuum_exact_agreement():
    """F8.1: Verifies RK4 numerical integrator matches exact vacuum closed-form trajectory."""
    cfg = BallisticConfig(air_density=0.0, muzzle_velocity=80.0, dynamic_expansion=False)
    calc = BallisticCalculator(cfg)

    t_eval = 0.75
    s_num, v_num = calc.compute_flight_distance_and_speed(t_eval)
    drop_num = calc.compute_vertical_drop(t_eval)

    exact_s = 80.0 * t_eval
    exact_v = 80.0
    exact_drop = 0.5 * 9.81 * (t_eval**2)

    assert s_num == pytest.approx(exact_s, abs=1e-5)
    assert v_num == pytest.approx(exact_v, abs=1e-5)
    assert drop_num == pytest.approx(exact_drop, abs=1e-5)


def test_rk4_trajectory_point_generation():
    """F8.2: Verifies integrate_trajectory returns complete state trajectory points."""
    calc = BallisticCalculator()
    points = calc.integrate_trajectory(duration=0.5, dt=0.01)

    assert len(points) == 51
    assert isinstance(points[0], TrajectoryPoint)
    assert points[0].t == 0.0
    assert points[0].pos == (0.0, 0.0, 0.0)
    assert points[0].speed == 80.0
    assert points[0].drop == 0.0

    # Projectile must move forward (+Z) and drop down (+drop)
    last_pt = points[-1]
    assert last_pt.t == 0.5
    assert last_pt.pos[2] > 20.0
    assert last_pt.pos[1] < 0.0
    assert last_pt.drop > 0.5
    assert last_pt.speed < 80.0


def test_newton_raphson_stationary_intercept():
    """F10.1: Verifies intercept solver for stationary target at 40m."""
    calc = BallisticCalculator()
    target = TargetState(pos_3d=(0.0, 2.0, 40.0), vel_3d=(0.0, 0.0, 0.0), speed_kmh=0.0)
    sol = calc.solve_intercept(target)

    assert sol.reachable is True
    assert 0.40 < sol.t_intercept < 0.85
    assert sol.lead_pos_3d == (0.0, 2.0, 40.0)
    assert sol.aim_pan_deg == pytest.approx(90.0, abs=0.2)  # Boresight center pan
    assert sol.aim_tilt_deg > 90.0  # Tilted up for gravity compensation
    assert sol.drop_m > 0.80
    assert sol.residual_m < 0.02


def test_newton_raphson_crossing_target_lead():
    """F10.2: Verifies lead calculation for 200 km/h (55.56 m/s) orthogonal flyby."""
    calc = BallisticCalculator()
    vx_mps = 200.0 / 3.6  # 55.56 m/s
    target = TargetState(pos_3d=(-10.0, 2.0, 30.0), vel_3d=(vx_mps, 0.0, 0.0), speed_kmh=200.0)
    sol = calc.solve_intercept(target)

    assert sol.reachable is True
    assert sol.t_intercept > 0.3
    # Drone traveled right: lead X must be > -10
    assert sol.lead_pos_3d[0] > -10.0
    # Turret pan must lead to the right (> 90 deg)
    assert sol.aim_pan_deg > 90.0
    assert sol.target_travel_m > 15.0


def test_tactical_hud_pixel_projection_and_ray_inversion():
    """F11.1: Verifies 3D-to-pixel projection and pixel-to-3D ray inversion."""
    calc = BallisticCalculator()
    frame_size = (640, 480)

    # Center target at (0, 0, 30m) -> Screen center (320, 240)
    px, py = calc.project_3d_to_pixel((0.0, 0.0, 30.0), frame_shape=frame_size)
    assert px == 320
    assert py == 240

    # Target right and up: (4.0, 2.0, 30.0) -> px > 320, py < 240
    px_ru, py_ru = calc.project_3d_to_pixel((4.0, 2.0, 30.0), frame_shape=frame_size)
    assert px_ru > 320
    assert py_ru < 240

    # Invert back to 3D ray at 30m distance
    ray_3d = calc.pixel_to_3d_ray((px_ru, py_ru), distance_m=30.0, frame_shape=frame_size)
    assert ray_3d[0] == pytest.approx(4.0, abs=0.1)
    assert ray_3d[1] == pytest.approx(2.0, abs=0.1)
    assert ray_3d[2] == pytest.approx(30.0, abs=0.1)
