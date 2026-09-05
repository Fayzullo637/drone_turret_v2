"""Physics Benchmarks & Analytical Reference Solvers for Drone Turret v2.

Provides exact closed-form analytical solutions and high-precision reference RK4 numerical
integrators to establish ground-truth baselines for ballistics and intercept algorithms.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Tuple
import numpy as np


class BallisticsBenchmarks:
    """Mathematical reference solutions for projectile ballistics and target intercept."""

    STANDARD_RHO = 1.225  # Air density at sea level (kg/m^3)
    STANDARD_G = 9.81     # Gravitational acceleration (m/s^2)

    @classmethod
    def vacuum_trajectory(
        cls,
        v0: float,
        theta_deg: float,
        target_x: float,
        g: float = STANDARD_G,
        mass: float = 0.6,
    ) -> Dict[str, float]:
        """Analytical closed-form ballistic trajectory in a vacuum (zero drag).
        
        Equations of motion:
        x(t) = v0 * cos(theta) * t
        y(t) = v0 * sin(theta) * t - 0.5 * g * t^2
        vx(t) = v0 * cos(theta)
        vy(t) = v0 * sin(theta) - g * t
        """
        theta_rad = math.radians(theta_deg)
        vx0 = v0 * math.cos(theta_rad)
        vy0 = v0 * math.sin(theta_rad)

        if vx0 <= 0.0:
            raise ValueError("Horizontal muzzle velocity component must be positive")

        t_flight = target_x / vx0
        y_drop = vy0 * t_flight - 0.5 * g * (t_flight**2)
        vx_final = vx0
        vy_final = vy0 - g * t_flight
        v_final = math.sqrt(vx_final**2 + vy_final**2)

        # Conservation of Mechanical Energy check
        e_initial = 0.5 * mass * (v0**2)
        e_final = 0.5 * mass * (v_final**2) + mass * g * y_drop

        return {
            "t_flight": t_flight,
            "y_drop": y_drop,
            "vx": vx_final,
            "vy": vy_final,
            "v_final": v_final,
            "e_initial": e_initial,
            "e_final": e_final,
            "energy_delta": abs(e_final - e_initial),
        }

    @classmethod
    def quadratic_drag_terminal_velocity(
        cls,
        mass: float,
        cd: float,
        area: float,
        rho: float = STANDARD_RHO,
        g: float = STANDARD_G,
    ) -> float:
        """Asymptotic terminal fall velocity under quadratic aerodynamic drag:
        
        v_term = sqrt((2 * m * g) / (rho * Cd * A))
        """
        if cd <= 0 or area <= 0 or mass <= 0:
            raise ValueError("mass, cd, and area must be positive non-zero values")
        return math.sqrt((2.0 * mass * g) / (rho * cd * area))

    @classmethod
    def rk4_step_3d(
        cls,
        state: np.ndarray,
        dt: float,
        mass: float,
        cd: float,
        area: float,
        rho: float = STANDARD_RHO,
        g: float = STANDARD_G,
    ) -> np.ndarray:
        """Single 4th-Order Runge-Kutta integration step for 3D projectile flight.
        
        State vector: [x, y, z, vx, vy, vz]
        d(pos)/dt = vel
        d(vel)/dt = -(1/(2*m)) * rho * Cd * A * |v| * vel + [0, -g, 0]
        """
        k_drag = 0.5 * rho * cd * area / mass

        def derivatives(s: np.ndarray) -> np.ndarray:
            pos = s[0:3]
            vel = s[3:6]
            v_mag = np.linalg.norm(vel)
            acc = -k_drag * v_mag * vel
            acc[1] -= g  # Gravity acts along -Y (vertical down)
            return np.concatenate([vel, acc])

        k1 = derivatives(state)
        k2 = derivatives(state + 0.5 * dt * k1)
        k3 = derivatives(state + 0.5 * dt * k2)
        k4 = derivatives(state + dt * k3)

        return state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    @classmethod
    def rk4_solve_flight(
        cls,
        v0: float,
        target_dist: float,
        theta_elev_deg: float = 0.0,
        psi_azimuth_deg: float = 0.0,
        mass: float = 0.60,
        cd: float = 1.20,
        area: float = 0.015,
        dt: float = 0.001,
        rho: float = STANDARD_RHO,
        g: float = STANDARD_G,
    ) -> Dict[str, float]:
        """Integrates 3D trajectory with quadratic drag until projectile reaches target_dist."""
        elev_rad = math.radians(theta_elev_deg)
        azim_rad = math.radians(psi_azimuth_deg)

        # Initial velocity vector
        vx0 = v0 * math.cos(elev_rad) * math.sin(azim_rad)
        vy0 = v0 * math.sin(elev_rad)
        vz0 = v0 * math.cos(elev_rad) * math.cos(azim_rad)

        state = np.array([0.0, 0.0, 0.0, vx0, vy0, vz0], dtype=np.float64)
        t = 0.0
        max_time = 10.0

        while t < max_time:
            dist_current = np.linalg.norm(state[0:3])
            if dist_current >= target_dist and t > 0:
                break
            state = cls.rk4_step_3d(state, dt, mass, cd, area, rho, g)
            t += dt

        speed_final = float(np.linalg.norm(state[3:6]))
        return {
            "t_flight": t,
            "x": float(state[0]),
            "y": float(state[1]),
            "z": float(state[2]),
            "vx": float(state[3]),
            "vy": float(state[4]),
            "vz": float(state[5]),
            "v_final": speed_final,
            "y_drop": float(state[1]),
        }

    @classmethod
    def reference_intercept_solver(
        cls,
        target_pos_3d: Tuple[float, float, float],
        target_vel_3d: Tuple[float, float, float],
        target_acc_3d: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        v0: float = 80.0,
        mass: float = 0.60,
        cd: float = 1.20,
        area: float = 0.015,
        max_iter: int = 8,
        tol: float = 1e-3,
    ) -> Dict[str, Any]:
        """Calculates intercept time t_int and drop-compensated aim vector via iterative root-finding.
        
        Target Position at time t:
        P_t(t) = P0 + V0*t + 0.5*A0*t^2
        """
        px, py, pz = target_pos_3d
        vx, vy, vz = target_vel_3d
        ax, ay, az = target_acc_3d

        # Initial flight time estimation from nominal muzzle velocity
        d0 = math.sqrt(px**2 + py**2 + pz**2)
        if d0 <= 0.01:
            return {
                "reachable": True,
                "t_intercept": 0.0,
                "lead_pos_3d": (px, py, pz),
                "aim_pan_deg": 90.0,
                "aim_tilt_deg": 90.0,
                "drop_m": 0.0,
            }

        t_int = d0 / max(1.0, v0)

        for _ in range(max_iter):
            # Target predicted position at t_int
            tgt_x = px + vx * t_int + 0.5 * ax * (t_int**2)
            tgt_y = py + vy * t_int + 0.5 * ay * (t_int**2)
            tgt_z = pz + vz * t_int + 0.5 * az * (t_int**2)
            dist_tgt = math.sqrt(tgt_x**2 + tgt_y**2 + tgt_z**2)

            # Solve projectile flight for distance dist_tgt
            flight = cls.rk4_solve_flight(
                v0=v0, target_dist=dist_tgt, mass=mass, cd=cd, area=area, dt=0.005
            )
            t_flight = flight["t_flight"]

            err = t_flight - t_int
            if abs(err) < tol:
                break
            # Relaxation step
            t_int = 0.5 * t_int + 0.5 * t_flight

        # Final predicted target position
        tgt_x = px + vx * t_int + 0.5 * ax * (t_int**2)
        tgt_y = py + vy * t_int + 0.5 * ay * (t_int**2)
        tgt_z = pz + vz * t_int + 0.5 * az * (t_int**2)

        # Drop compensation
        # Gravity drop = y_tgt - (flight_y with 0 elevation)
        drop_comp = 0.5 * cls.STANDARD_G * (t_int**2)  # approximate or from RK4
        aim_y = tgt_y + drop_comp

        # Spherical angle conversion for pan/tilt
        # Turret frame: 90 deg pan = straight ahead (+Z), 90 deg tilt = horizontal level (Y=0)
        pan_azimuth_rad = math.atan2(tgt_x, max(0.1, tgt_z))
        horiz_dist = math.hypot(tgt_x, tgt_z)
        tilt_elevation_rad = math.atan2(aim_y, max(0.1, horiz_dist))

        pan_deg = 90.0 + math.degrees(pan_azimuth_rad)
        tilt_deg = 90.0 + math.degrees(tilt_elevation_rad)

        return {
            "reachable": t_int < 5.0 and dist_tgt <= 100.0,
            "t_intercept": float(t_int),
            "lead_pos_3d": (float(tgt_x), float(tgt_y), float(tgt_z)),
            "aim_pan_deg": float(np.clip(pan_deg, 0.0, 180.0)),
            "aim_tilt_deg": float(np.clip(tilt_deg, 0.0, 180.0)),
            "drop_m": float(drop_comp),
        }
