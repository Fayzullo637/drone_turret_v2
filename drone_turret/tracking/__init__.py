"""Tracking and state estimation subsystem for drone_turret_v2.

Exports:
- KalmanPredictiveTracker: 6-state CA predictive Kalman filter
- MultiTargetKalmanTracker: Multi-track ensemble manager
- TargetState: Kinematic state representation
- FilterStatus: Tracking lifecycle states
- KalmanConfig, FilterConfig: Filter configuration parameters
- Math utility functions: compute_transition_matrix, compute_process_noise, etc.
"""

from drone_turret.tracking.kalman_filter import (
    FilterConfig,
    FilterStatus,
    KalmanConfig,
    KalmanPredictiveTracker,
    MultiTargetKalmanTracker,
    TargetState,
    compute_process_noise,
    compute_transition_matrix,
    focal_length_from_hfov,
    pixel_to_camera_3d,
    velocity_to_speed_kmh,
)

__all__ = [
    "KalmanPredictiveTracker",
    "MultiTargetKalmanTracker",
    "TargetState",
    "FilterStatus",
    "KalmanConfig",
    "FilterConfig",
    "compute_transition_matrix",
    "compute_process_noise",
    "focal_length_from_hfov",
    "pixel_to_camera_3d",
    "velocity_to_speed_kmh",
]
