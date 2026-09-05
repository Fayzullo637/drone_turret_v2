"""
Dual-Axis Discrete PID Controller for Pan & Tilt Interceptor Turret Servos.

Features:
1. Target Lead Point Aiming (pan_lead, tilt_lead).
2. Deadband Filtering (default ±0.3°) to prevent continuous jitter and servo heating.
3. Anti-Windup Clamping on the Integral Accumulator.
4. Low-pass Filtered Derivative Term (alpha = 0.7) to attenuate high-frequency noise.
5. Slew Rate Limiting (max angular change per step, default 10.0°/step).
6. Absolute Angular Clamping to physical servo range [0.0°, 180.0°].
7. Full state tracking & telemetry export for Web HUD visualization.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple

# Default tuned gains & parameters
DEFAULT_KP = 0.12
DEFAULT_KI = 0.005
DEFAULT_KD = 0.03
DEFAULT_DEADBAND_DEG = 0.3
DEFAULT_MAX_STEP_DEG = 10.0
DEFAULT_I_MAX = 5.0
DEFAULT_DERIVATIVE_ALPHA = 0.7
SERVO_MIN_ANGLE_DEG = 0.0
SERVO_MAX_ANGLE_DEG = 180.0
SERVO_HOME_PAN_DEG = 90.0
SERVO_HOME_TILT_DEG = 90.0


@dataclass
class PIDTelemetry:
    """Detailed telemetry for a single PID axis."""
    target_deg: float = 90.0
    current_deg: float = 90.0
    raw_error_deg: float = 0.0
    filtered_error_deg: float = 0.0
    p_term: float = 0.0
    i_term: float = 0.0
    d_term: float = 0.0
    step_output_deg: float = 0.0
    integral_accum: float = 0.0


class DiscretePID:
    """
    Single-axis robust discrete PID controller with anti-windup, deadband,
    derivative filtering, and slew-rate limiting.
    """

    def __init__(
        self,
        kp: float = DEFAULT_KP,
        ki: float = DEFAULT_KI,
        kd: float = DEFAULT_KD,
        deadband: float = DEFAULT_DEADBAND_DEG,
        max_step: float = DEFAULT_MAX_STEP_DEG,
        i_max: float = DEFAULT_I_MAX,
        derivative_alpha: float = DEFAULT_DERIVATIVE_ALPHA,
        min_angle: float = SERVO_MIN_ANGLE_DEG,
        max_angle: float = SERVO_MAX_ANGLE_DEG,
        initial_angle: float = 90.0,
    ) -> None:
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.deadband = float(deadband)
        self.max_step = float(max_step)
        self.i_max = float(i_max)
        self.derivative_alpha = float(derivative_alpha)
        self.min_angle = float(min_angle)
        self.max_angle = float(max_angle)

        self.current_angle = float(initial_angle)
        self.target_angle = float(initial_angle)

        # Internal state
        self._integral_accum: float = 0.0
        self._prev_filtered_error: float = 0.0
        self._filtered_derivative: float = 0.0
        self._is_first_step: bool = True

        # Last computed metrics for telemetry
        self.telemetry = PIDTelemetry(
            target_deg=self.target_angle,
            current_deg=self.current_angle,
        )

    def set_gains(
        self,
        kp: Optional[float] = None,
        ki: Optional[float] = None,
        kd: Optional[float] = None,
        deadband: Optional[float] = None,
        max_step: Optional[float] = None,
    ) -> None:
        """Dynamically updates PID gains during runtime."""
        if kp is not None:
            self.kp = float(kp)
        if ki is not None:
            self.ki = float(ki)
        if kd is not None:
            self.kd = float(kd)
        if deadband is not None:
            self.deadband = max(0.0, float(deadband))
        if max_step is not None:
            self.max_step = max(0.1, float(max_step))

    def reset(self, initial_angle: Optional[float] = None) -> None:
        """Resets integral accumulator and derivative memory."""
        self._integral_accum = 0.0
        self._prev_filtered_error = 0.0
        self._filtered_derivative = 0.0
        self._is_first_step = True
        if initial_angle is not None:
            self.current_angle = max(self.min_angle, min(self.max_angle, float(initial_angle)))
            self.target_angle = self.current_angle

    def update(self, target_deg: float, dt: float) -> float:
        """
        Executes one discrete PID update step.
        
        Args:
            target_deg: Target angle in degrees.
            dt: Time step in seconds since last update.
            
        Returns:
            New commanded absolute angle in [min_angle, max_angle].
        """
        # Guard against invalid inputs
        if math.isnan(target_deg) or math.isinf(target_deg):
            return self.current_angle

        # Clamp requested target to valid physical domain
        self.target_angle = max(self.min_angle, min(self.max_angle, float(target_deg)))

        # Compute raw error
        raw_error = self.target_angle - self.current_angle

        # 1. Apply deadband filtering
        if abs(raw_error) <= self.deadband:
            filtered_error = 0.0
        else:
            filtered_error = raw_error

        # 2. Proportional term
        p_term = self.kp * filtered_error

        # 3. Integral term with Anti-Windup Clamping
        if dt > 0.0:
            self._integral_accum += filtered_error * dt
            # Clamp accumulator
            self._integral_accum = max(-self.i_max, min(self.i_max, self._integral_accum))
            i_term = self.ki * self._integral_accum
        else:
            i_term = self.ki * self._integral_accum

        # 4. Filtered Derivative term
        if dt > 0.0 and not self._is_first_step:
            raw_derivative = (filtered_error - self._prev_filtered_error) / dt
            # Exponential low-pass filter: D = alpha * D_raw + (1 - alpha) * D_prev
            self._filtered_derivative = (
                self.derivative_alpha * raw_derivative
                + (1.0 - self.derivative_alpha) * self._filtered_derivative
            )
            d_term = self.kd * self._filtered_derivative
        else:
            d_term = 0.0
            self._is_first_step = False

        self._prev_filtered_error = filtered_error

        # 5. Total control signal & Slew Rate Limiting
        control_signal = p_term + i_term + d_term
        step_output = max(-self.max_step, min(self.max_step, control_signal))

        # 6. Absolute Output Clamping to physical range [0, 180]
        self.current_angle = max(
            self.min_angle,
            min(self.max_angle, self.current_angle + step_output),
        )

        # Update telemetry
        self.telemetry = PIDTelemetry(
            target_deg=round(self.target_angle, 2),
            current_deg=round(self.current_angle, 2),
            raw_error_deg=round(raw_error, 2),
            filtered_error_deg=round(filtered_error, 2),
            p_term=round(p_term, 4),
            i_term=round(i_term, 4),
            d_term=round(d_term, 4),
            step_output_deg=round(step_output, 3),
            integral_accum=round(self._integral_accum, 4),
        )

        return self.current_angle


class TurretController:
    """
    Dual-Axis Pan & Tilt Turret PID Controller.
    
    Coordinates independent horizontal (Pan) and vertical (Tilt) PID control loops,
    driving turret servos to aim directly at the calculated ballistic lead intercept point.
    """

    def __init__(
        self,
        pan_kp: float = DEFAULT_KP,
        pan_ki: float = DEFAULT_KI,
        pan_kd: float = DEFAULT_KD,
        tilt_kp: float = DEFAULT_KP,
        tilt_ki: float = DEFAULT_KI,
        tilt_kd: float = DEFAULT_KD,
        deadband_deg: float = DEFAULT_DEADBAND_DEG,
        max_step_deg: float = DEFAULT_MAX_STEP_DEG,
        home_pan_deg: float = SERVO_HOME_PAN_DEG,
        home_tilt_deg: float = SERVO_HOME_TILT_DEG,
    ) -> None:
        self.home_pan_deg = float(home_pan_deg)
        self.home_tilt_deg = float(home_tilt_deg)

        self.pan_pid = DiscretePID(
            kp=pan_kp,
            ki=pan_ki,
            kd=pan_kd,
            deadband=deadband_deg,
            max_step=max_step_deg,
            initial_angle=self.home_pan_deg,
        )

        self.tilt_pid = DiscretePID(
            kp=tilt_kp,
            ki=tilt_ki,
            kd=tilt_kd,
            deadband=deadband_deg,
            max_step=max_step_deg,
            initial_angle=self.home_tilt_deg,
        )

    @property
    def current_angles(self) -> Tuple[float, float]:
        """Returns current (pan_deg, tilt_deg)."""
        return (self.pan_pid.current_angle, self.tilt_pid.current_angle)

    @property
    def target_angles(self) -> Tuple[float, float]:
        """Returns current target (pan_lead_deg, tilt_lead_deg)."""
        return (self.pan_pid.target_angle, self.tilt_pid.target_angle)

    def update_lead_target(
        self,
        target_pan_deg: float,
        target_tilt_deg: float,
        dt: float,
    ) -> Tuple[float, float]:
        """
        Calculates discrete PID step and returns (cmd_pan_deg, cmd_tilt_deg) clamped to [0, 180].
        
        Args:
            target_pan_deg: Ballistic lead azimuth angle in degrees [0, 180].
            target_tilt_deg: Ballistic lead elevation angle in degrees [0, 180].
            dt: Time step in seconds.
            
        Returns:
            Tuple of (cmd_pan_deg, cmd_tilt_deg) ready for serial transmission.
        """
        dt_clamped = max(0.0, min(1.0, float(dt))) if not math.isnan(dt) else 0.033

        cmd_pan = self.pan_pid.update(target_pan_deg, dt_clamped)
        cmd_tilt = self.tilt_pid.update(target_tilt_deg, dt_clamped)

        return (round(cmd_pan, 2), round(cmd_tilt, 2))

    def home(self, dt: float = 0.05) -> Tuple[float, float]:
        """Commands turret servos back to neutral home angles (90°, 90°)."""
        return self.update_lead_target(self.home_pan_deg, self.home_tilt_deg, dt)

    def reset(
        self,
        pan_deg: Optional[float] = None,
        tilt_deg: Optional[float] = None,
    ) -> None:
        """Resets both PID controllers to given or home angles."""
        p = pan_deg if pan_deg is not None else self.home_pan_deg
        t = tilt_deg if tilt_deg is not None else self.home_tilt_deg
        self.pan_pid.reset(p)
        self.tilt_pid.reset(t)

    def set_current_angles(self, pan_deg: float, tilt_deg: float) -> None:
        """Sets internal current angle tracking without stepping PID."""
        self.pan_pid.current_angle = max(SERVO_MIN_ANGLE_DEG, min(SERVO_MAX_ANGLE_DEG, float(pan_deg)))
        self.tilt_pid.current_angle = max(SERVO_MIN_ANGLE_DEG, min(SERVO_MAX_ANGLE_DEG, float(tilt_deg)))

    def set_gains(
        self,
        kp: Optional[float] = None,
        ki: Optional[float] = None,
        kd: Optional[float] = None,
        deadband: Optional[float] = None,
        max_step: Optional[float] = None,
    ) -> None:
        """Updates gains uniformly for both pan and tilt."""
        self.pan_pid.set_gains(kp=kp, ki=ki, kd=kd, deadband=deadband, max_step=max_step)
        self.tilt_pid.set_gains(kp=kp, ki=ki, kd=kd, deadband=deadband, max_step=max_step)

    def get_telemetry(self) -> Dict[str, Any]:
        """Returns comprehensive telemetry dictionary for both axes."""
        return {
            "pan": {
                "target_deg": self.pan_pid.telemetry.target_deg,
                "current_deg": self.pan_pid.telemetry.current_deg,
                "raw_error_deg": self.pan_pid.telemetry.raw_error_deg,
                "filtered_error_deg": self.pan_pid.telemetry.filtered_error_deg,
                "p_term": self.pan_pid.telemetry.p_term,
                "i_term": self.pan_pid.telemetry.i_term,
                "d_term": self.pan_pid.telemetry.d_term,
                "step_output_deg": self.pan_pid.telemetry.step_output_deg,
            },
            "tilt": {
                "target_deg": self.tilt_pid.telemetry.target_deg,
                "current_deg": self.tilt_pid.telemetry.current_deg,
                "raw_error_deg": self.tilt_pid.telemetry.raw_error_deg,
                "filtered_error_deg": self.tilt_pid.telemetry.filtered_error_deg,
                "p_term": self.tilt_pid.telemetry.p_term,
                "i_term": self.tilt_pid.telemetry.i_term,
                "d_term": self.tilt_pid.telemetry.d_term,
                "step_output_deg": self.tilt_pid.telemetry.step_output_deg,
            },
            "lead_angles": {
                "pan": self.pan_pid.telemetry.target_deg,
                "tilt": self.tilt_pid.telemetry.target_deg,
            },
            "current_angles": {
                "pan": self.pan_pid.telemetry.current_deg,
                "tilt": self.tilt_pid.telemetry.current_deg,
            },
        }


# Backwards compatibility alias
DualAxisPIDController = TurretController
