"""
Sensors package: LiDAR driver & Passive Optical Distance Estimator.
"""

from drone_turret.sensors.distance import (
    DEFAULT_CAMERA_HFOV_DEG,
    DEFAULT_DRONE_SIZE_M,
    DRONE_PRESETS,
    DistanceEstimator,
    DistanceResult,
    compute_focal_length_px,
    compute_hfov_deg,
    get_drone_size,
)
from drone_turret.sensors.lidar import (
    DEFAULT_MAX_DISTANCE_M,
    DEFAULT_MIN_DISTANCE_M,
    DEFAULT_MIN_STRENGTH,
    FRAME_HEADER,
    FRAME_LENGTH,
    LidarParser,
    LidarReading,
    LidarSerialReader,
)

__all__ = [
    "LidarReading",
    "LidarParser",
    "LidarSerialReader",
    "FRAME_HEADER",
    "FRAME_LENGTH",
    "DEFAULT_MIN_STRENGTH",
    "DEFAULT_MIN_DISTANCE_M",
    "DEFAULT_MAX_DISTANCE_M",
    "DistanceEstimator",
    "DistanceResult",
    "DRONE_PRESETS",
    "DEFAULT_DRONE_SIZE_M",
    "DEFAULT_CAMERA_HFOV_DEG",
    "compute_focal_length_px",
    "compute_hfov_deg",
    "get_drone_size",
]
