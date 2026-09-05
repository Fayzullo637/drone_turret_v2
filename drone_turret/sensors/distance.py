"""
Passive Optical Distance Estimator & Sensor Fusion Module.

Implements:
1. Pinhole camera geometry for passive optical ranging:
   distance = (known_drone_size * focal_length) / bbox_width_pixels
2. Standard military / commercial drone size presets (DJI Mavic 3, Shahed-136, FPV Quad, MQ-9).
3. Camera Horizontal Field of View (HFOV) to focal length conversion.
4. Seamless fusion / fallback logic: LiDAR primary, Optical BBox secondary.
5. 3D turret-centric coordinate recovery & km/h speed calculation.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

# Predefined physical drone dimensions (wingspan / diagonal width in meters)
DRONE_PRESETS: Dict[str, float] = {
    "DJI Mavic 3": 0.35,
    "DJI Mavic": 0.35,
    "mavic": 0.35,
    "Shahed-136": 2.50,
    "Shahed": 2.50,
    "shahed": 2.50,
    "FPV": 0.22,
    "FPV Quad": 0.22,
    "fpv": 0.22,
    "MQ-9": 20.0,
    "MQ-9 Reaper": 20.0,
    "mq9": 20.0,
    "Generic Drone": 0.35,
    "default": 0.35,
}

DEFAULT_DRONE_SIZE_M = 0.35
DEFAULT_CAMERA_HFOV_DEG = 70.0
DEFAULT_MIN_DISTANCE_M = 0.1
DEFAULT_MAX_DISTANCE_M = 500.0


def compute_focal_length_px(image_width: int, hfov_deg: float = DEFAULT_CAMERA_HFOV_DEG) -> float:
    """
    Computes camera focal length in pixels from image width and horizontal field of view.
    
    Formula:
        f_px = (image_width / 2) / tan(HFOV / 2)
    """
    if image_width <= 0:
        raise ValueError(f"Image width must be positive, got {image_width}")
    if not (0.0 < hfov_deg < 180.0):
        raise ValueError(f"HFOV must be in (0, 180) degrees, got {hfov_deg}")

    hfov_rad = math.radians(hfov_deg)
    return (image_width / 2.0) / math.tan(hfov_rad / 2.0)


def compute_hfov_deg(image_width: int, focal_length_px: float) -> float:
    """
    Computes camera horizontal field of view in degrees from focal length in pixels.
    """
    if image_width <= 0 or focal_length_px <= 0:
        raise ValueError("Image width and focal length must be positive")
    
    hfov_rad = 2.0 * math.atan((image_width / 2.0) / focal_length_px)
    return math.degrees(hfov_rad)


def get_drone_size(preset_name_or_custom: Optional[str] = None, custom_size_m: Optional[float] = None) -> float:
    """
    Resolves drone physical size in meters from preset name or custom dimension.
    """
    if custom_size_m is not None and custom_size_m > 0:
        return float(custom_size_m)

    if preset_name_or_custom is not None:
        key = preset_name_or_custom.strip()
        if key in DRONE_PRESETS:
            return DRONE_PRESETS[key]
        # Case-insensitive search
        for preset_key, size in DRONE_PRESETS.items():
            if preset_key.lower() == key.lower():
                return size

    return DEFAULT_DRONE_SIZE_M


@dataclass
class DistanceResult:
    """Full distance estimation output with telemetry metadata."""
    distance_m: float
    source: str  # 'lidar' or 'optical'
    confidence: float
    drone_size_m: float
    focal_length_px: float
    bbox_width_px: int


class DistanceEstimator:
    """
    Unified distance estimation and sensor fusion engine.
    
    Provides high-accuracy ranging by fusing active LiDAR rangefinder measurements
    with passive optical pinhole camera geometry fallback.
    """

    def __init__(
        self,
        default_preset: str = "DJI Mavic 3",
        camera_hfov_deg: float = DEFAULT_CAMERA_HFOV_DEG,
        min_distance_m: float = DEFAULT_MIN_DISTANCE_M,
        max_distance_m: float = DEFAULT_MAX_DISTANCE_M,
        custom_drone_size_m: Optional[float] = None,
    ) -> None:
        self.default_preset = default_preset
        self.camera_hfov_deg = camera_hfov_deg
        self.min_distance_m = min_distance_m
        self.max_distance_m = max_distance_m
        self.custom_drone_size_m = custom_drone_size_m

    def estimate_optical_distance(
        self,
        bbox: Tuple[int, int, int, int] | Tuple[float, float, float, float],
        frame_shape: Tuple[int, int] | Tuple[int, int, int],
        preset: Optional[str] = None,
        custom_size_m: Optional[float] = None,
        hfov_deg: Optional[float] = None,
    ) -> float:
        """
        Calculates distance in meters using pinhole optical perspective model:
            distance = (known_drone_size * focal_length) / bbox_width_pixels
        """
        # frame_shape can be (H, W) or (H, W, C)
        img_h, img_w = frame_shape[0], frame_shape[1]
        x1, y1, x2, y2 = bbox
        bbox_w = abs(x2 - x1)
        
        # Guard against degenerate bounding box
        bbox_w = max(1.0, float(bbox_w))

        target_size_m = get_drone_size(
            preset or self.default_preset,
            custom_size_m if custom_size_m is not None else self.custom_drone_size_m,
        )

        fov = hfov_deg if hfov_deg is not None else self.camera_hfov_deg
        f_px = compute_focal_length_px(img_w, fov)

        raw_dist = (target_size_m * f_px) / bbox_w
        clamped_dist = max(self.min_distance_m, min(self.max_distance_m, raw_dist))
        return round(clamped_dist, 3)

    def get_distance(
        self,
        bbox: Tuple[int, int, int, int] | Tuple[float, float, float, float],
        frame_shape: Tuple[int, int] | Tuple[int, int, int],
        lidar_dist: Optional[float] = None,
        drone_preset: Optional[str] = None,
        custom_size: Optional[float] = None,
    ) -> Tuple[float, str]:
        """
        Fuses LiDAR and Optical ranging.
        
        Returns:
            (distance_meters, source_name: 'lidar' | 'optical')
        """
        # 1. Primary: Use LiDAR if valid and within physical bounds
        if lidar_dist is not None and lidar_dist > 0:
            if self.min_distance_m <= lidar_dist <= self.max_distance_m:
                return (round(float(lidar_dist), 3), "lidar")

        # 2. Secondary: Fallback to optical pinhole ranging
        optical_dist = self.estimate_optical_distance(
            bbox=bbox,
            frame_shape=frame_shape,
            preset=drone_preset,
            custom_size_m=custom_size,
        )
        return (optical_dist, "optical")

    def calculate_3d_position(
        self,
        pixel_xy: Tuple[float, float],
        distance_m: float,
        frame_shape: Tuple[int, int] | Tuple[int, int, int],
        hfov_deg: Optional[float] = None,
    ) -> Tuple[float, float, float]:
        """
        Transforms 2D screen coordinates and distance into 3D turret-centric coordinates:
            +X: Right (meters)
            +Y: Up (meters)
            +Z: Downrange / Forward (meters)
        """
        img_h, img_w = frame_shape[0], frame_shape[1]
        cx, cy = img_w / 2.0, img_h / 2.0
        
        fov = hfov_deg if hfov_deg is not None else self.camera_hfov_deg
        fx = compute_focal_length_px(img_w, fov)
        fy = fx  # Assume square pixels

        u, v = pixel_xy
        X = ((u - cx) * distance_m) / fx
        Y = -((v - cy) * distance_m) / fy  # Inverted since pixel y goes down, world Y goes up
        Z = distance_m

        return (round(X, 3), round(Y, 3), round(Z, 3))

    def calculate_speed_kmh(
        self,
        pixel_vel_xy: Tuple[float, float],
        distance_m: float,
        frame_shape: Tuple[int, int] | Tuple[int, int, int],
        range_rate_mps: float = 0.0,
        hfov_deg: Optional[float] = None,
    ) -> float:
        """
        Computes 3D kinematic speed in km/h from 2D pixel velocity and target distance:
            v_X = (vx_px * d) / fx
            v_Y = -(vy_px * d) / fy
            v_Z = range_rate_mps (rate of change of distance along optical axis)
            speed_kmh = sqrt(v_X^2 + v_Y^2 + v_Z^2) * 3.6
        """
        img_h, img_w = frame_shape[0], frame_shape[1]
        fov = hfov_deg if hfov_deg is not None else self.camera_hfov_deg
        fx = compute_focal_length_px(img_w, fov)
        fy = fx

        vx_px, vy_px = pixel_vel_xy
        v_x = (vx_px * distance_m) / fx
        v_y = -(vy_px * distance_m) / fy
        v_z = range_rate_mps

        speed_mps = math.sqrt(v_x**2 + v_y**2 + v_z**2)
        return round(speed_mps * 3.6, 2)
