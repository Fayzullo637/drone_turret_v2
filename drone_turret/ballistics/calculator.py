"""
Ballistic Calculator & Intercept Solver Module for drone_turret_v2.

Implements:
1. Dynamic expanding aerodynamic drag net model.
2. 4th-order Runge-Kutta (RK4) numerical trajectory integrator for 3D/1D flight.
3. Newton-Raphson root-finding algorithm for intercept time determination.
4. Pan/Tilt servo lead aiming angle computation with vertical gravity + drag drop compensation.
5. 3D-to-2D image lead point projection for tactical HUD crosshair overlays.
"""

from dataclasses import dataclass, field
import math
from typing import Any, Dict, List, Optional, Tuple, Union

import numpy as np


@dataclass
class BallisticConfig:
    """Configuration parameters for ballistic trajectory and intercept calculations."""

    # Projectile pneumatic & aerodynamic parameters
    muzzle_velocity: float = 80.0  # Initial launch speed v0 (m/s) [40.0 - 120.0]
    projectile_mass: float = 0.35  # Projectile + net mass m (kg) [0.1 - 2.0]
    cd_initial: float = 0.45  # Drag coeff before net deployment (canister)
    cd_max: float = 1.35  # Drag coeff after full net mesh deployment [1.1 - 1.5]
    area_initial: float = 0.0015  # Frontal area before deployment (m^2, canister)
    area_max: float = 0.0080  # Effective net mesh twine & weight frontal area (m^2)
    tau_deploy: float = 0.15  # Deployment time constant tau (seconds)
    dynamic_expansion: bool = True  # Enable time-dependent Cd(t) and A(t) expansion

    # Environment parameters
    air_density: float = 1.225  # Standard air density rho (kg/m^3)
    gravity: float = 9.81  # Gravitational acceleration g (m/s^2)
    wind_velocity_3d: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # Wind vector (wx, wy, wz) in m/s, turret-centric

    # Numerical integration parameters
    dt_step: float = 0.005  # RK4 integration time step h (seconds)
    max_flight_time: float = 3.0  # Max projectile flight simulation time (seconds)

    # Operational range limits
    min_effective_range_m: float = 1.0  # Min engagement distance (meters)
    max_effective_range_m: float = 75.0  # Max engagement distance (meters)

    # Turret servo geometry & limits
    pan_center_deg: float = 90.0  # Boresight neutral pan angle (degrees)
    tilt_center_deg: float = 90.0  # Level neutral tilt angle (degrees)
    pan_min_deg: float = 0.0  # Minimum pan servo angle limit
    pan_max_deg: float = 180.0  # Maximum pan servo angle limit
    tilt_min_deg: float = 0.0  # Minimum tilt servo angle limit
    tilt_max_deg: float = 180.0  # Maximum tilt servo angle limit

    # Camera optical properties for 2D projection
    camera_hfov_deg: float = 70.0  # Camera horizontal field of view (degrees)
    camera_frame_size: Tuple[int, int] = (640, 480)  # (width, height) in pixels


@dataclass
class TargetState:
    """Kinematic state of tracked drone target."""

    pos_2d: Tuple[float, float] = (0.0, 0.0)  # (x, y) centroid in screen pixels
    vel_2d: Tuple[float, float] = (0.0, 0.0)  # (vx, vy) velocity in px/s
    acc_2d: Tuple[float, float] = (0.0, 0.0)  # (ax, ay) acceleration in px/s^2
    pos_3d: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # (X, Y, Z) in meters
    vel_3d: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # (vX, vY, vZ) in m/s
    speed_kmh: float = 0.0  # Target speed in km/h
    is_coasting: bool = False  # True if state is coasting during detection loss
    coast_frames: int = 0  # Number of consecutive coasting frames
    acc_3d: Optional[Tuple[float, float, float]] = None  # (aX, aY, aZ) in m/s^2


@dataclass
class TrajectoryPoint:
    """Snapshot of projectile state along its flight path."""

    t: float  # Elapsed time from launch (s)
    pos: Tuple[float, float, float]  # (X, Y, Z) position in meters
    vel: Tuple[float, float, float]  # (vX, vY, vZ) velocity in m/s
    speed: float  # Scalar speed |v| in m/s
    cd: float  # Drag coefficient Cd(t)
    area: float  # Frontal area A(t) in m^2
    drop: float  # Vertical drop from bore line in meters


@dataclass
class InterceptSolution:
    """Computed firing and intercept guidance solution."""

    reachable: bool  # True if target can be intercepted within constraints
    t_intercept: float  # Flight time to intercept in seconds
    lead_pos_3d: Tuple[float, float, float]  # Target projected position at t_intercept (m)
    aim_pan_deg: float  # Commanded pan servo angle [0, 180]
    aim_tilt_deg: float  # Commanded tilt servo angle [0, 180]
    lead_pixel_xy: Tuple[int, int]  # (x, y) lead reticle pixel coordinates on HUD
    drop_m: float  # Vertical gravity + drag drop compensation in meters
    flight_distance_m: float = 0.0  # Distance traveled by projectile at t_intercept
    target_travel_m: float = 0.0  # Distance traveled by drone between t=0 and t_int
    iterations: int = 0  # Newton-Raphson root-finder iteration count
    residual_m: float = 0.0  # Final root-finding distance residual |s_net - R_d|


class BallisticCalculator:
    """
    High-precision ballistic trajectory integrator and intercept solver.

    Uses 4th-order Runge-Kutta numerical integration with variable aerodynamic drag
    and dynamic net expansion, coupled with a Newton-Raphson root finder to determine
    the optimal lead aim point and servo angles.
    """

    def __init__(self, config: Optional[BallisticConfig] = None) -> None:
        """Initialize calculator with given or default configuration."""
        self.config = config if config is not None else BallisticConfig()

    def update_config(self, **kwargs: Any) -> None:
        """Update configuration parameters dynamically at runtime."""
        for key, value in kwargs.items():
            if hasattr(self.config, key):
                setattr(self.config, key, value)
            else:
                raise AttributeError(f"BallisticConfig has no attribute '{key}'")

    # =========================================================================
    # Aerodynamic Model
    # =========================================================================

    def cd_at_time(self, t: float) -> float:
        """
        Calculate time-dependent drag coefficient Cd(t).

        Canister expands exponentially:
        Cd(t) = Cd_0 + (Cd_max - Cd_0) * (1 - exp(-t / tau_deploy))
        """
        if not self.config.dynamic_expansion or t <= 0.0:
            return self.config.cd_initial if t <= 0.0 and self.config.dynamic_expansion else self.config.cd_max

        tau = max(self.config.tau_deploy, 1e-6)
        expansion = 1.0 - math.exp(-t / tau)
        return self.config.cd_initial + (self.config.cd_max - self.config.cd_initial) * expansion

    def area_at_time(self, t: float) -> float:
        """
        Calculate time-dependent frontal net area A(t).

        A(t) = A_0 + (A_max - A_0) * (1 - exp(-t / tau_deploy))
        """
        if not self.config.dynamic_expansion or t <= 0.0:
            return self.config.area_initial if t <= 0.0 and self.config.dynamic_expansion else self.config.area_max

        tau = max(self.config.tau_deploy, 1e-6)
        expansion = 1.0 - math.exp(-t / tau)
        return self.config.area_initial + (self.config.area_max - self.config.area_initial) * expansion

    def drag_acceleration(self, t: float, vel: Union[np.ndarray, Tuple[float, float, float]]) -> np.ndarray:
        """
        Compute aerodynamic drag deceleration vector with wind compensation.

        Uses wind-relative velocity for drag calculation:
            v_rel = v_projectile - v_wind
            a_drag = - (1 / (2 * m)) * rho * Cd(t) * A(t) * ||v_rel|| * v_rel
        """
        v_arr = np.asarray(vel, dtype=np.float64)
        v_wind = np.asarray(self.config.wind_velocity_3d, dtype=np.float64)
        v_rel = v_arr - v_wind  # Velocity relative to air mass
        speed_rel = float(np.linalg.norm(v_rel))
        if speed_rel < 1e-9:
            return np.zeros_like(v_arr)

        cd = self.cd_at_time(t)
        area = self.area_at_time(t)
        mass = max(self.config.projectile_mass, 1e-6)
        rho = max(self.config.air_density, 0.0)

        coeff = 0.5 * rho * cd * area / mass
        return -coeff * speed_rel * v_rel

    # =========================================================================
    # RK4 Numerical Integrator
    # =========================================================================

    def rk4_step_3d(
        self,
        t: float,
        pos: Union[np.ndarray, Tuple[float, float, float]],
        vel: Union[np.ndarray, Tuple[float, float, float]],
        dt: float,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Perform a single 4th-order Runge-Kutta numerical integration step in 3D.

        State vector: pos = [X, Y, Z], vel = [vX, vY, vZ]
        Differential equations:
            d(pos)/dt = vel
            d(vel)/dt = a_drag(t, vel) + g_vec
        """
        p_arr = np.asarray(pos, dtype=np.float64)
        v_arr = np.asarray(vel, dtype=np.float64)
        g_vec = np.array([0.0, -self.config.gravity, 0.0], dtype=np.float64)

        # k1
        v1 = v_arr
        a1 = self.drag_acceleration(t, v1) + g_vec

        # k2
        t2 = t + 0.5 * dt
        v2 = v_arr + 0.5 * dt * a1
        a2 = self.drag_acceleration(t2, v2) + g_vec

        # k3
        t3 = t + 0.5 * dt
        v3 = v_arr + 0.5 * dt * a2
        a3 = self.drag_acceleration(t3, v3) + g_vec

        # k4
        t4 = t + dt
        v4 = v_arr + dt * a3
        a4 = self.drag_acceleration(t4, v4) + g_vec

        # Accumulate
        new_pos = p_arr + (dt / 6.0) * (v1 + 2.0 * v2 + 2.0 * v3 + v4)
        new_vel = v_arr + (dt / 6.0) * (a1 + 2.0 * a2 + 2.0 * a3 + a4)

        return new_pos, new_vel

    def rk4_step_1d(self, t: float, s: float, v: float, dt: float) -> Tuple[float, float]:
        """
        Perform a single 1D RK4 step along the projectile trajectory path.

        State: s (distance along path), v (speed along path)
        ds/dt = v
        dv/dt = -0.5 * rho * Cd(t) * A(t) * v^2 / m
        """
        mass = max(self.config.projectile_mass, 1e-6)
        rho = max(self.config.air_density, 0.0)

        def accel(t_eval: float, v_eval: float) -> float:
            if v_eval <= 0.0:
                return 0.0
            cd = self.cd_at_time(t_eval)
            area = self.area_at_time(t_eval)
            return -0.5 * rho * cd * area * (v_eval**2) / mass

        # k1
        v1 = v
        a1 = accel(t, v1)

        # k2
        t2 = t + 0.5 * dt
        v2 = max(0.0, v + 0.5 * dt * a1)
        a2 = accel(t2, v2)

        # k3
        t3 = t + 0.5 * dt
        v3 = max(0.0, v + 0.5 * dt * a2)
        a3 = accel(t3, v3)

        # k4
        t4 = t + dt
        v4 = max(0.0, v + dt * a3)
        a4 = accel(t4, v4)

        new_s = s + (dt / 6.0) * (v1 + 2.0 * v2 + 2.0 * v3 + v4)
        new_v = max(0.0, v + (dt / 6.0) * (a1 + 2.0 * a2 + 2.0 * a3 + a4))

        return new_s, new_v

    def compute_flight_distance_and_speed(self, t: float, dt: Optional[float] = None) -> Tuple[float, float]:
        """
        Integrate 1D projectile distance and remaining speed at flight time t.

        Fast, optimized RK4 implementation.
        Returns: (distance_meters, speed_mps)
        """
        if t <= 0.0:
            return 0.0, float(self.config.muzzle_velocity)

        v0 = float(self.config.muzzle_velocity)
        mass = max(self.config.projectile_mass, 1e-6)
        rho = max(self.config.air_density, 0.0)

        # Vacuum optimization
        if rho <= 1e-12:
            return v0 * t, v0

        h = dt if dt is not None else self.config.dt_step
        num_steps = max(1, int(math.ceil(t / h)))
        step_dt = t / num_steps

        cd0 = self.config.cd_initial
        cd_max = self.config.cd_max
        a0 = self.config.area_initial
        a_max = self.config.area_max
        tau = max(self.config.tau_deploy, 1e-6)
        dynamic = self.config.dynamic_expansion

        c0 = 0.5 * rho / mass
        s = 0.0
        v = v0
        curr_t = 0.0
        half_dt = 0.5 * step_dt
        dt_div_6 = step_dt / 6.0

        for _ in range(num_steps):
            if dynamic:
                exp1 = math.exp(-curr_t / tau)
                k1 = c0 * (cd0 + (cd_max - cd0) * (1.0 - exp1)) * (a0 + (a_max - a0) * (1.0 - exp1))

                t_mid = curr_t + half_dt
                exp_mid = math.exp(-t_mid / tau)
                k_mid = c0 * (cd0 + (cd_max - cd0) * (1.0 - exp_mid)) * (a0 + (a_max - a0) * (1.0 - exp_mid))

                t_end = curr_t + step_dt
                exp_end = math.exp(-t_end / tau)
                k_end = c0 * (cd0 + (cd_max - cd0) * (1.0 - exp_end)) * (a0 + (a_max - a0) * (1.0 - exp_end))
            else:
                k1 = k_mid = k_end = c0 * cd_max * a_max

            a1 = -k1 * v * v
            v2 = max(0.0, v + half_dt * a1)
            a2 = -k_mid * v2 * v2
            v3 = max(0.0, v + half_dt * a2)
            a3 = -k_mid * v3 * v3
            v4 = max(0.0, v + step_dt * a3)
            a4 = -k_end * v4 * v4

            s += dt_div_6 * (v + 2.0 * v2 + 2.0 * v3 + v4)
            v = max(0.0, v + dt_div_6 * (a1 + 2.0 * a2 + 2.0 * a3 + a4))
            curr_t += step_dt
            if v <= 0.0:
                break

        return s, v

    def compute_vertical_drop(self, t: float, dt: Optional[float] = None) -> float:
        """
        Calculate vertical drop (gravity + drag) of projectile fired horizontally for time t.

        Returns: drop magnitude in meters (positive value).
        """
        if t <= 0.0:
            return 0.0

        g = self.config.gravity
        rho = max(self.config.air_density, 0.0)

        # Vacuum optimization
        if rho <= 1e-12:
            return 0.5 * g * (t**2)

        v0 = float(self.config.muzzle_velocity)
        mass = max(self.config.projectile_mass, 1e-6)
        h = dt if dt is not None else self.config.dt_step
        num_steps = max(1, int(math.ceil(t / h)))
        step_dt = t / num_steps

        cd0 = self.config.cd_initial
        cd_max = self.config.cd_max
        a0 = self.config.area_initial
        a_max = self.config.area_max
        tau = max(self.config.tau_deploy, 1e-6)
        dynamic = self.config.dynamic_expansion

        c0 = 0.5 * rho / mass
        y = 0.0
        vy = 0.0
        vz = v0
        curr_t = 0.0
        half_dt = 0.5 * step_dt
        dt_div_6 = step_dt / 6.0

        for _ in range(num_steps):
            if dynamic:
                exp1 = math.exp(-curr_t / tau)
                k1 = c0 * (cd0 + (cd_max - cd0) * (1.0 - exp1)) * (a0 + (a_max - a0) * (1.0 - exp1))

                t_mid = curr_t + half_dt
                exp_mid = math.exp(-t_mid / tau)
                k_mid = c0 * (cd0 + (cd_max - cd0) * (1.0 - exp_mid)) * (a0 + (a_max - a0) * (1.0 - exp_mid))

                t_end = curr_t + step_dt
                exp_end = math.exp(-t_end / tau)
                k_end = c0 * (cd0 + (cd_max - cd0) * (1.0 - exp_end)) * (a0 + (a_max - a0) * (1.0 - exp_end))
            else:
                k1 = k_mid = k_end = c0 * cd_max * a_max

            spd1 = math.sqrt(vz * vz + vy * vy)
            az1 = -k1 * spd1 * vz
            ay1 = -g - k1 * spd1 * vy

            vz2 = vz + half_dt * az1
            vy2 = vy + half_dt * ay1
            spd2 = math.sqrt(vz2 * vz2 + vy2 * vy2)
            az2 = -k_mid * spd2 * vz2
            ay2 = -g - k_mid * spd2 * vy2

            vz3 = vz + half_dt * az2
            vy3 = vy + half_dt * ay2
            spd3 = math.sqrt(vz3 * vz3 + vy3 * vy3)
            az3 = -k_mid * spd3 * vz3
            ay3 = -g - k_mid * spd3 * vy3

            vz4 = vz + step_dt * az3
            vy4 = vy + step_dt * ay3
            spd4 = math.sqrt(vz4 * vz4 + vy4 * vy4)
            az4 = -k_end * spd4 * vz4
            ay4 = -g - k_end * spd4 * vy4

            y += dt_div_6 * (vy + 2.0 * vy2 + 2.0 * vy3 + vy4)
            vy += dt_div_6 * (ay1 + 2.0 * ay2 + 2.0 * ay3 + ay4)
            vz = max(0.0, vz + dt_div_6 * (az1 + 2.0 * az2 + 2.0 * az3 + az4))
            curr_t += step_dt

        return max(0.0, -y)

    def integrate_trajectory(
        self,
        initial_pos: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        initial_vel: Optional[Tuple[float, float, float]] = None,
        duration: Optional[float] = None,
        dt: Optional[float] = None,
    ) -> List[TrajectoryPoint]:
        """
        Simulate full 3D projectile trajectory over specified duration.

        Returns list of TrajectoryPoint snapshots.
        """
        total_time = duration if duration is not None else self.config.max_flight_time
        h = dt if dt is not None else self.config.dt_step

        if initial_vel is None:
            v_init = np.array([0.0, 0.0, self.config.muzzle_velocity], dtype=np.float64)
        else:
            v_init = np.array(initial_vel, dtype=np.float64)

        p = np.array(initial_pos, dtype=np.float64)
        v = v_init.copy()
        t = 0.0

        points: List[TrajectoryPoint] = []

        spd = float(np.linalg.norm(v))
        points.append(
            TrajectoryPoint(
                t=0.0,
                pos=(float(p[0]), float(p[1]), float(p[2])),
                vel=(float(v[0]), float(v[1]), float(v[2])),
                speed=spd,
                cd=self.cd_at_time(0.0),
                area=self.area_at_time(0.0),
                drop=0.0,
            )
        )

        num_steps = max(1, int(math.ceil(total_time / h)))
        step_dt = total_time / num_steps

        for _ in range(num_steps):
            p, v = self.rk4_step_3d(t, p, v, step_dt)
            t += step_dt
            spd = float(np.linalg.norm(v))
            drop = float(initial_pos[1] - p[1])

            points.append(
                TrajectoryPoint(
                    t=round(t, 6),
                    pos=(float(p[0]), float(p[1]), float(p[2])),
                    vel=(float(v[0]), float(v[1]), float(v[2])),
                    speed=spd,
                    cd=self.cd_at_time(t),
                    area=self.area_at_time(t),
                    drop=drop,
                )
            )

            # Terminate if hit ground or stopped
            if p[1] < -50.0 or spd < 0.1:
                break

        return points

    # =========================================================================
    # Kinematics & Target Extrapolation
    # =========================================================================

    def predict_target_pos_3d(self, target: TargetState, t: float) -> np.ndarray:
        """
        Extrapolate drone 3D position at future time t:
        p_d(t) = p_d(0) + v_d * t + 0.5 * a_d * t^2
        """
        p0 = np.asarray(target.pos_3d, dtype=np.float64)
        v0 = np.asarray(target.vel_3d, dtype=np.float64)

        if target.acc_3d is not None:
            a0 = np.asarray(target.acc_3d, dtype=np.float64)
        else:
            a0 = np.zeros(3, dtype=np.float64)

        return p0 + v0 * t + 0.5 * a0 * (t**2)

    def predict_target_vel_3d(self, target: TargetState, t: float) -> np.ndarray:
        """
        Extrapolate drone 3D velocity at future time t:
        v_d(t) = v_d(0) + a_d * t
        """
        v0 = np.asarray(target.vel_3d, dtype=np.float64)
        if target.acc_3d is not None:
            a0 = np.asarray(target.acc_3d, dtype=np.float64)
        else:
            a0 = np.zeros(3, dtype=np.float64)

        return v0 + a0 * t

    # =========================================================================
    # Newton-Raphson Intercept Solver
    # =========================================================================

    def solve_intercept(
        self,
        target: TargetState,
        max_iterations: int = 25,
        tolerance_m: float = 0.01,
    ) -> InterceptSolution:
        """
        Solve for optimal intercept lead point and time using Newton-Raphson root finding.

        Root equation:
            F(t) = s_net(t) - ||p_d(t)|| = 0
            F'(t) = v_net(t) - (p_d(t) . v_d(t)) / ||p_d(t)||

        Returns InterceptSolution with lead position, drop compensation, servo angles,
        and HUD screen crosshair coordinates.
        """
        p0 = np.asarray(target.pos_3d, dtype=np.float64)
        r0 = float(np.linalg.norm(p0))

        # Check range feasibility
        if r0 < self.config.min_effective_range_m or r0 > self.config.max_effective_range_m:
            return self._unreachable_solution(target, r0)

        # Initial estimate: t0 = distance / muzzle_velocity
        v0 = max(self.config.muzzle_velocity, 1.0)
        t_curr = r0 / v0
        t_curr = max(0.01, min(t_curr, self.config.max_flight_time))

        converged = False
        final_residual = 0.0
        iters = 0

        for i in range(max_iterations):
            iters = i + 1

            s_net, v_net = self.compute_flight_distance_and_speed(t_curr)
            p_target = self.predict_target_pos_3d(target, t_curr)
            v_target = self.predict_target_vel_3d(target, t_curr)
            r_target = float(np.linalg.norm(p_target))

            if r_target < 1e-4:
                r_target = 1e-4

            f_val = s_net - r_target
            final_residual = abs(f_val)

            if final_residual <= tolerance_m:
                converged = True
                break

            f_prime = v_net - float(np.dot(p_target, v_target)) / r_target

            if abs(f_prime) < 1e-5:
                f_prime = 1.0 if f_prime >= 0 else -1.0

            step = f_val / f_prime
            step = max(-0.5, min(0.5, step))
            t_next = t_curr - step

            if t_next <= 0.001 or t_next > self.config.max_flight_time:
                t_next = max(0.01, min(t_curr * 0.8 if step > 0 else t_curr * 1.2, self.config.max_flight_time))

            if abs(t_next - t_curr) < 1e-6:
                converged = True
                t_curr = t_next
                break

            t_curr = t_next

        # If Newton didn't converge within tolerance, execute bounded bisection fallback
        if not converged or final_residual > tolerance_m * 5.0:
            t_curr, converged, final_residual = self._bisection_fallback(target, tolerance_m)

        t_int = max(0.001, t_curr)
        p_int = self.predict_target_pos_3d(target, t_int)
        r_int = float(np.linalg.norm(p_int))

        if r_int > self.config.max_effective_range_m or t_int > self.config.max_flight_time or not converged:
            return self._unreachable_solution(target, r_int, t_int=t_int)

        # Drop compensation
        drop_m = self.compute_vertical_drop(t_int)

        x_aim = float(p_int[0])
        y_aim = float(p_int[1] + drop_m)
        z_aim = float(p_int[2])

        # Aiming servo angles
        pan_deg, tilt_deg = self.compute_aim_angles((x_aim, y_aim, z_aim))

        # Screen HUD lead point projection
        lead_px, lead_py = self.project_3d_to_pixel((x_aim, y_aim, z_aim))

        # Flight distance and drone travel
        flight_dist, _ = self.compute_flight_distance_and_speed(t_int)
        drone_travel = float(np.linalg.norm(p_int - p0))

        # Check reachability constraints
        reachable = (
            self.config.pan_min_deg <= pan_deg <= self.config.pan_max_deg
            and self.config.tilt_min_deg <= tilt_deg <= self.config.tilt_max_deg
            and z_aim > 0.05
        )

        return InterceptSolution(
            reachable=reachable,
            t_intercept=round(t_int, 4),
            lead_pos_3d=(float(p_int[0]), float(p_int[1]), float(p_int[2])),
            aim_pan_deg=round(pan_deg, 2),
            aim_tilt_deg=round(tilt_deg, 2),
            lead_pixel_xy=(lead_px, lead_py),
            drop_m=round(drop_m, 4),
            flight_distance_m=round(flight_dist, 2),
            target_travel_m=round(drone_travel, 2),
            iterations=iters,
            residual_m=round(final_residual, 5),
        )

    def _bisection_fallback(self, target: TargetState, tolerance_m: float) -> Tuple[float, bool, float]:
        """Bounded bisection fallback if Newton-Raphson does not achieve desired tolerance."""
        t_low = 0.005
        t_high = self.config.max_flight_time

        def eval_f(t_eval: float) -> float:
            s, _ = self.compute_flight_distance_and_speed(t_eval)
            p = self.predict_target_pos_3d(target, t_eval)
            return s - float(np.linalg.norm(p))

        f_low = eval_f(t_low)
        f_high = eval_f(t_high)

        if f_low * f_high > 0.0:
            samples = np.linspace(t_low, t_high, 10)  # Reduced from 25 to 10
            best_t = t_low
            best_abs_f = abs(f_low)
            found_bracket = False

            prev_t = t_low
            prev_f = f_low
            for s_t in samples[1:]:
                curr_f = eval_f(s_t)
                if abs(curr_f) < best_abs_f:
                    best_abs_f = abs(curr_f)
                    best_t = s_t
                if prev_f * curr_f <= 0.0:
                    t_low = prev_t
                    t_high = s_t
                    found_bracket = True
                    break
                prev_t = s_t
                prev_f = curr_f

            if not found_bracket:
                return float(best_t), (best_abs_f < tolerance_m * 10), float(best_abs_f)

        t_mid = 0.5 * (t_low + t_high)
        for _ in range(15):  # Reduced from 30 to 15
            t_mid = 0.5 * (t_low + t_high)
            f_mid = eval_f(t_mid)
            if abs(f_mid) <= tolerance_m or (t_high - t_low) < 1e-5:
                return float(t_mid), True, float(abs(f_mid))
            if eval_f(t_low) * f_mid <= 0.0:
                t_high = t_mid
            else:
                t_low = t_mid

        return float(t_mid), (abs(eval_f(t_mid)) < tolerance_m * 5), float(abs(eval_f(t_mid)))

    def _unreachable_solution(
        self,
        target: TargetState,
        distance_m: float,
        t_int: float = 0.0,
    ) -> InterceptSolution:
        """Construct fallback InterceptSolution when target is out of range or unreachable."""
        p0 = (float(target.pos_3d[0]), float(target.pos_3d[1]), float(target.pos_3d[2]))
        pan_deg, tilt_deg = self.compute_aim_angles(p0)
        lead_px, lead_py = self.project_3d_to_pixel(p0)

        return InterceptSolution(
            reachable=False,
            t_intercept=round(t_int, 4),
            lead_pos_3d=p0,
            aim_pan_deg=round(pan_deg, 2),
            aim_tilt_deg=round(tilt_deg, 2),
            lead_pixel_xy=(lead_px, lead_py),
            drop_m=0.0,
            flight_distance_m=0.0,
            target_travel_m=0.0,
            iterations=0,
            residual_m=round(distance_m, 4),
        )

    # =========================================================================
    # Aim Angles and Camera Projection
    # =========================================================================

    def compute_aim_angles(self, aim_pos_3d: Union[Tuple[float, float, float], np.ndarray]) -> Tuple[float, float]:
        """
        Convert 3D turret-centric target/aim coordinates into Pan/Tilt servo angles.

        Turret Boresight (+Z) corresponds to (Pan=90.0 deg, Tilt=90.0 deg).
        +X is to the right (Pan increases).
        +Y is upwards (Tilt increases).

        Returns: (pan_deg, tilt_deg) clamped to configured servo limits [0, 180].
        """
        x, y, z = float(aim_pos_3d[0]), float(aim_pos_3d[1]), float(aim_pos_3d[2])

        z_safe = max(z, 0.01) if z >= 0 else min(z, -0.01)
        ground_dist = math.sqrt(x * x + z * z)

        pan_offset_deg = math.degrees(math.atan2(x, z_safe))
        pan_deg = self.config.pan_center_deg + pan_offset_deg

        tilt_offset_deg = math.degrees(math.atan2(y, max(ground_dist, 1e-4)))
        tilt_deg = self.config.tilt_center_deg + tilt_offset_deg

        pan_clamped = max(self.config.pan_min_deg, min(self.config.pan_max_deg, pan_deg))
        tilt_clamped = max(self.config.tilt_min_deg, min(self.config.tilt_max_deg, tilt_deg))

        return float(pan_clamped), float(tilt_clamped)

    def project_3d_to_pixel(
        self,
        pos_3d: Union[Tuple[float, float, float], np.ndarray],
        frame_shape: Optional[Tuple[int, int]] = None,
        hfov_deg: Optional[float] = None,
    ) -> Tuple[int, int]:
        """
        Project 3D turret-centric point (X, Y, Z) onto 2D camera pixel coordinates (x, y).

        Standard pinhole camera projection:
            f_px = (width / 2) / tan(HFOV / 2)
            x_lead = c_x + f_px * (X / Z)
            y_lead = c_y - f_px * (Y / Z)  (minus sign because +Y is up, image +y is down)

        Returns: (pixel_x, pixel_y) rounded to integer.
        """
        width, height = frame_shape if frame_shape is not None else self.config.camera_frame_size
        fov = hfov_deg if hfov_deg is not None else self.config.camera_hfov_deg

        cx = width / 2.0
        cy = height / 2.0

        fov_rad = math.radians(max(1.0, min(170.0, fov)))
        focal_length_px = (width / 2.0) / math.tan(fov_rad / 2.0)

        x, y, z = float(pos_3d[0]), float(pos_3d[1]), float(pos_3d[2])
        z_safe = max(z, 0.05)

        px = int(round(cx + focal_length_px * (x / z_safe)))
        py = int(round(cy - focal_length_px * (y / z_safe)))

        return px, py

    def pixel_to_3d_ray(
        self,
        pixel_xy: Tuple[float, float],
        distance_m: float,
        frame_shape: Optional[Tuple[int, int]] = None,
        hfov_deg: Optional[float] = None,
    ) -> Tuple[float, float, float]:
        """
        Inverse projection: Convert 2D pixel coordinates and distance into 3D point (X, Y, Z).

        X = (pixel_x - c_x) * distance / f_px
        Y = -(pixel_y - c_y) * distance / f_px
        Z = distance
        """
        width, height = frame_shape if frame_shape is not None else self.config.camera_frame_size
        fov = hfov_deg if hfov_deg is not None else self.config.camera_hfov_deg

        cx = width / 2.0
        cy = height / 2.0

        fov_rad = math.radians(max(1.0, min(170.0, fov)))
        focal_length_px = (width / 2.0) / math.tan(fov_rad / 2.0)

        px, py = pixel_xy
        d = max(distance_m, 0.01)

        x = ((px - cx) * d) / focal_length_px
        y = -((py - cy) * d) / focal_length_px
        z = d

        return float(x), float(y), float(z)
