"""Tier 1 Unit Tests: RK4 Ballistic Integrator & Dynamic Drag Model (Features F8, F9).

Verifies 4th-order Runge-Kutta numerical integration, energy conservation in vacuum,
analytical closed-form matching, variable drag Cd in [1.1, 1.5], and terminal velocity convergence.
"""

from __future__ import annotations

import math
import numpy as np
import pytest

from tests.fixtures.physics_benchmarks import BallisticsBenchmarks


def test_rk4_vacuum_energy_conservation(physics_benchmarks):
    """T1.4.1: Verifies total mechanical energy is conserved in vacuum flight."""
    v0 = 80.0
    theta_deg = 15.0
    target_dist = 50.0

    res = physics_benchmarks.vacuum_trajectory(
        v0=v0, theta_deg=theta_deg, target_x=target_dist, g=9.81, mass=0.6
    )

    assert res["energy_delta"] < 1e-6
    assert res["t_flight"] > 0.0


def test_rk4_vacuum_vs_analytical_solution(physics_benchmarks):
    """T1.4.2: Compares RK4 numerical integration in vacuum to closed-form analytical solution."""
    v0 = 60.0
    target_dist = 50.0

    analytical = physics_benchmarks.vacuum_trajectory(
        v0=v0, theta_deg=0.0, target_x=target_dist, g=9.81
    )
    # RK4 with Cd = 0
    numerical = physics_benchmarks.rk4_solve_flight(
        v0=v0,
        target_dist=target_dist,
        theta_elev_deg=0.0,
        mass=0.60,
        cd=0.0,  # Vacuum
        area=0.015,
        dt=0.001,
    )

    assert numerical["t_flight"] == pytest.approx(analytical["t_flight"], abs=2e-3)
    assert numerical["y_drop"] == pytest.approx(analytical["y_drop"], abs=2e-2)


def test_rk4_aerodynamic_drag_slowdown(physics_benchmarks):
    """T1.4.3: Verifies aerodynamic drag reduces projectile forward velocity over flight."""
    v0 = 80.0
    target_dist = 50.0

    vacuum = physics_benchmarks.rk4_solve_flight(
        v0=v0, target_dist=target_dist, cd=0.0, mass=0.6, dt=0.001
    )
    dragged = physics_benchmarks.rk4_solve_flight(
        v0=v0, target_dist=target_dist, cd=1.2, mass=0.6, area=0.015, dt=0.001
    )

    # Flight time with drag is longer than vacuum
    assert dragged["t_flight"] > vacuum["t_flight"]
    # Final speed with drag is less than muzzle velocity
    assert dragged["v_final"] < v0
    # Gravity drop with drag is greater because flight took longer
    assert abs(dragged["y_drop"]) > abs(vacuum["y_drop"])


def test_rk4_expanding_net_drag_cd_range(physics_benchmarks):
    """T1.4.4: Verifies drag increases monotonically as Cd increases from 1.1 to 1.5."""
    target_dist = 40.0
    flight_cd11 = physics_benchmarks.rk4_solve_flight(
        v0=75.0, target_dist=target_dist, cd=1.1, mass=0.7, area=0.015
    )
    flight_cd15 = physics_benchmarks.rk4_solve_flight(
        v0=75.0, target_dist=target_dist, cd=1.5, mass=0.7, area=0.015
    )

    assert flight_cd15["t_flight"] > flight_cd11["t_flight"]
    assert flight_cd15["v_final"] < flight_cd11["v_final"]
    assert abs(flight_cd15["y_drop"]) > abs(flight_cd11["y_drop"])


def test_rk4_terminal_velocity_asymptotic_convergence(physics_benchmarks):
    """T1.4.5: Verifies falling projectile converges on theoretical terminal velocity."""
    mass = 0.60
    cd = 1.20
    area = 0.015
    v_term_theory = physics_benchmarks.quadratic_drag_terminal_velocity(
        mass=mass, cd=cd, area=area
    )

    # Simulate purely vertical drop from high altitude
    state = np.array([0.0, 1000.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
    dt = 0.005
    for _ in range(3000):  # 15 seconds of freefall
        state = physics_benchmarks.rk4_step_3d(
            state, dt=dt, mass=mass, cd=cd, area=area
        )

    v_fall = abs(state[4])  # |vy|
    assert v_fall == pytest.approx(v_term_theory, rel=1e-2)


def test_rk4_step_size_stability(physics_benchmarks):
    """T1.4.6: Verifies RK4 numerical stability and accuracy across different dt steps."""
    res_coarse = physics_benchmarks.rk4_solve_flight(
        v0=80.0, target_dist=50.0, dt=0.005
    )
    res_fine = physics_benchmarks.rk4_solve_flight(
        v0=80.0, target_dist=50.0, dt=0.0005
    )

    assert res_coarse["t_flight"] == pytest.approx(res_fine["t_flight"], rel=1e-2)
    assert res_coarse["y_drop"] == pytest.approx(res_fine["y_drop"], rel=1e-2)
