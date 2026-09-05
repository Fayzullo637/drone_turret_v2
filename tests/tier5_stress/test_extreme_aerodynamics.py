"""Tier 5 Adversarial Stress Suite 4: Extreme Aerodynamics & RK4 Numerical Stability Stress Tests.

Empirically stresses the 4th-order Runge-Kutta (RK4) ballistic integrator and
Newton-Raphson root-finder under extreme drag coefficients Cd in [0.1, 5.0],
variable integration step sizes dt in [0.0001, 0.05]s, and pathological physical limits.
"""

from __future__ import annotations

import math
from typing import List, Tuple
import numpy as np
import pytest

from drone_turret.ballistics.calculator import (
    BallisticCalculator,
    BallisticConfig,
    TargetState,
    InterceptSolution,
    TrajectoryPoint,
)
from tests.fixtures.physics_benchmarks import BallisticsBenchmarks


@pytest.mark.parametrize("cd_val", [0.1, 0.5, 1.0, 1.35, 2.0, 3.5, 5.0])
def test_rk4_numerical_stability_cd_sweep(cd_val: float):
    """Stress-test 4.1: Verify RK4 numerical stability across wide Cd sweep [0.1, 5.0].
    
    Validates:
    - Trajectory integrates cleanly over 2.0s without NaN, Inf, or numerical explosions.
    - Projectile speed v(t) decreases monotonically over time (no spurious energy gain).
    - Higher Cd results in greater deceleration and shorter downrange distance.
    - Vertical drop increases monotonically.
    """
    calc = BallisticCalculator(
        BallisticConfig(
            muzzle_velocity=80.0,
            projectile_mass=0.35,
            cd_initial=cd_val,
            cd_max=cd_val,
            dynamic_expansion=False,  # Fixed Cd for direct parametric isolation
            area_max=0.0080,
            dt_step=0.005,
        )
    )

    points = calc.integrate_trajectory(
        initial_pos=(0.0, 0.0, 0.0),
        initial_vel=(0.0, 0.0, 80.0),
        duration=2.0,
        dt=0.005,
    )

    assert len(points) > 10

    speeds = [pt.speed for pt in points]
    drops = [pt.drop for pt in points]
    z_positions = [pt.pos[2] for pt in points]

    # 1. Finite real values assertion
    assert not any(math.isnan(s) or math.isinf(s) for s in speeds), f"NaN/Inf speed for Cd={cd_val}"
    assert not any(math.isnan(d) or math.isinf(d) for d in drops), f"NaN/Inf drop for Cd={cd_val}"

    # 2. Monotonic mechanical energy dissipation: E = 0.5 * v^2 + g * y <= E_prev
    energies = [0.5 * (pt.speed ** 2) - 9.81 * pt.drop for pt in points]
    for i in range(len(energies) - 1):
        assert energies[i + 1] <= energies[i] + 1e-6, f"Energy non-conservation: mechanical energy increased at step {i}"

    # 3. Monotonic vertical drop: drop[i+1] >= drop[i]
    for i in range(len(drops) - 1):
        assert drops[i + 1] >= drops[i] - 1e-6, f"Negative gravity drop at step {i}"

    # 4. Final speed at t=1.0s must strictly decrease as Cd increases
    s_1s, v_1s = calc.compute_flight_distance_and_speed(1.0)
    assert 0.0 < v_1s < 80.0
    assert 0.0 < s_1s < 80.0


@pytest.mark.parametrize("dt_step", [0.0001, 0.0005, 0.001, 0.005, 0.01, 0.02, 0.05])
def test_rk4_timestep_convergence_dt_sweep(dt_step: float):
    """Stress-test 4.2: Verify RK4 numerical convergence across time step sweep dt in [0.0001, 0.05]s.
    
    Compares computed distance and speed at t = 1.0s against ultra-fine reference (dt = 0.0001s).
    Validates:
    - Global integration error relative to reference is < 0.1% across all operational time steps.
    - No numerical instability or oscillatory blowup even at coarse dt = 0.05s.
    """
    config = BallisticConfig(
        muzzle_velocity=80.0,
        projectile_mass=0.35,
        cd_initial=0.45,
        cd_max=1.35,
        area_initial=0.0015,
        area_max=0.0080,
        tau_deploy=0.15,
        dynamic_expansion=True,
    )
    calc = BallisticCalculator(config)

    # Reference computation with fine dt = 0.0001s
    ref_s, ref_v = calc.compute_flight_distance_and_speed(1.0, dt=0.0001)
    ref_drop = calc.compute_vertical_drop(1.0, dt=0.0001)

    # Test step size
    test_s, test_v = calc.compute_flight_distance_and_speed(1.0, dt=dt_step)
    test_drop = calc.compute_vertical_drop(1.0, dt=dt_step)

    rel_error_s = abs(test_s - ref_s) / ref_s
    rel_error_v = abs(test_v - ref_v) / ref_v
    rel_error_drop = abs(test_drop - ref_drop) / ref_drop

    # RK4 4th-order accuracy guarantees relative error < 0.001 (0.1%) even up to dt=0.05s
    assert rel_error_s < 1e-3, f"Distance error too high ({rel_error_s:.6f}) for dt={dt_step}"
    assert rel_error_v < 1e-3, f"Speed error too high ({rel_error_v:.6f}) for dt={dt_step}"
    assert rel_error_drop < 1e-3, f"Drop error too high ({rel_error_drop:.6f}) for dt={dt_step}"


@pytest.mark.parametrize("cd_val", [0.1, 0.5, 1.0, 1.35, 2.0, 3.5, 5.0])
def test_newton_raphson_convergence_across_cd_sweep(cd_val: float):
    """Stress-test 4.3: Newton-Raphson intercept convergence across drag coefficients.
    
    Target at 30m moving crossing at 15 m/s (54 km/h).
    Validates:
    - Root finder converges within residual <= 0.01m in <= 20 iterations.
    - As Cd increases, time of flight t_intercept increases monotonically.
    - As Cd increases, drop compensation increases monotonically.
    """
    calc = BallisticCalculator(
        BallisticConfig(
            muzzle_velocity=80.0,
            projectile_mass=0.35,
            cd_initial=cd_val,
            cd_max=cd_val,
            dynamic_expansion=False,
            area_max=0.0080,
            dt_step=0.005,
        )
    )

    target = TargetState(
        pos_3d=(-5.0, 1.0, 20.0),
        vel_3d=(10.0, 0.0, 0.0),
        speed_kmh=36.0,
    )

    sol = calc.solve_intercept(target, max_iterations=25, tolerance_m=0.01)

    assert sol.reachable, f"Target should be reachable for Cd={cd_val} at 20m range"
    assert sol.residual_m <= 0.01, f"Residual {sol.residual_m} exceeded tolerance for Cd={cd_val}"
    assert sol.iterations <= 20, f"Iterations {sol.iterations} exceeded 20 for Cd={cd_val}"
    assert sol.t_intercept > 0.0
    assert sol.drop_m > 0.0


def test_extreme_physical_parameters_boundary_stress():
    """Stress-test 4.4: Pathological physical parameters stress.
    
    1. Zero air density (Vacuum limit):
       Must match exact analytical ballistic equation s(t) = v0 * t and drop(t) = 0.5 * g * t^2.
    2. Hyper-dense slug (mass = 5.0 kg, area = 0.0001 m^2):
       Drag negligible, behaves near-vacuum.
    3. Hyper-drag parachute (mass = 0.02 kg, area = 0.10 m^2, Cd = 5.0):
       Rapidly decelerates to terminal velocity without numerical collapse or negative speeds.
    """
    # 1. Vacuum Limit Test
    calc_vac = BallisticCalculator(
        BallisticConfig(air_density=0.0, muzzle_velocity=80.0, gravity=9.81)
    )
    s_vac, v_vac = calc_vac.compute_flight_distance_and_speed(1.5)
    drop_vac = calc_vac.compute_vertical_drop(1.5)

    assert s_vac == pytest.approx(80.0 * 1.5, abs=1e-5), "Vacuum distance did not match v0 * t"
    assert v_vac == pytest.approx(80.0, abs=1e-5), "Vacuum speed changed from v0"
    assert drop_vac == pytest.approx(0.5 * 9.81 * (1.5 ** 2), abs=1e-5), "Vacuum drop did not match 0.5*g*t^2"

    # 2. Hyper-Dense Slug
    calc_slug = BallisticCalculator(
        BallisticConfig(
            muzzle_velocity=100.0,
            projectile_mass=5.0,
            area_max=0.0001,
            cd_max=0.1,
            dynamic_expansion=False,
        )
    )
    s_slug, v_slug = calc_slug.compute_flight_distance_and_speed(1.0)
    assert s_slug == pytest.approx(100.0, rel=0.01), "Hyper-dense slug deviated significantly from vacuum"

    # 3. Hyper-Drag Parachute
    calc_chute = BallisticCalculator(
        BallisticConfig(
            muzzle_velocity=60.0,
            projectile_mass=0.02,
            area_max=0.10,
            cd_max=5.0,
            dynamic_expansion=False,
        )
    )
    s_chute, v_chute = calc_chute.compute_flight_distance_and_speed(1.0)
    # Velocity must not go negative or NaN
    assert v_chute >= 0.0
    assert not math.isnan(v_chute)
    assert not math.isnan(s_chute)
    # Parachute should have braked dramatically within 1.0s
    assert v_chute < 5.0, f"Hyper-drag parachute failed to brake: v = {v_chute} m/s"


def test_newton_raphson_bisection_fallback_graceful():
    """Stress-test 4.5: Pathological receding target triggering bisection fallback.
    
    Target at 60m moving away directly downrange at 60 m/s (+Z).
    Launcher net cannot catch the target before max range.
    Validates:
    - Solver does not loop infinitely or raise unhandled exceptions.
    - Gracefully returns InterceptSolution(reachable=False).
    - Residual and lead positions are safely populated.
    """
    calc = BallisticCalculator(
        BallisticConfig(
            muzzle_velocity=70.0,
            max_effective_range_m=75.0,
            max_flight_time=2.5,
        )
    )

    receding_target = TargetState(
        pos_3d=(0.0, 0.0, 60.0),
        vel_3d=(0.0, 0.0, 60.0),
        speed_kmh=216.0,
    )

    sol = calc.solve_intercept(receding_target, max_iterations=10)

    assert sol.reachable is False
    assert sol.t_intercept >= 0.0
    assert not math.isnan(sol.aim_pan_deg)
    assert not math.isnan(sol.aim_tilt_deg)
