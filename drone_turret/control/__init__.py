"""
Control package: Dual-Axis Discrete PID Turret Controllers.
"""

from drone_turret.control.pid import (
    DEFAULT_DEADBAND_DEG,
    DEFAULT_DERIVATIVE_ALPHA,
    DEFAULT_I_MAX,
    DEFAULT_KD,
    DEFAULT_KI,
    DEFAULT_KP,
    DEFAULT_MAX_STEP_DEG,
    SERVO_HOME_PAN_DEG,
    SERVO_HOME_TILT_DEG,
    SERVO_MAX_ANGLE_DEG,
    SERVO_MIN_ANGLE_DEG,
    DiscretePID,
    DualAxisPIDController,
    PIDTelemetry,
    TurretController,
)

__all__ = [
    "DiscretePID",
    "TurretController",
    "DualAxisPIDController",
    "PIDTelemetry",
    "DEFAULT_KP",
    "DEFAULT_KI",
    "DEFAULT_KD",
    "DEFAULT_DEADBAND_DEG",
    "DEFAULT_MAX_STEP_DEG",
    "DEFAULT_I_MAX",
    "DEFAULT_DERIVATIVE_ALPHA",
    "SERVO_MIN_ANGLE_DEG",
    "SERVO_MAX_ANGLE_DEG",
    "SERVO_HOME_PAN_DEG",
    "SERVO_HOME_TILT_DEG",
]
