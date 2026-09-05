"""Tier 1 Unit Tests: Passive Optical Bounding Box Distance Estimator (Features F13, F7).

Verifies pinhole camera distance estimation D = (W_real * f) / w_pixel, drone size presets,
LiDAR priority with seamless optical fallback, and EMA noise smoothing.
"""

from __future__ import annotations

import math
from typing import Dict, Optional, Tuple
import pytest

from tests.fixtures.synthetic_video import TargetKinematics


class DistanceEstimator:
    """Estimates target distance using optical pinhole bounding box math and LiDAR fallback."""

    DEFAULT_PRESETS = {
        "dji_mini": 0.25,    # 25cm
        "dji_mavic": 0.40,   # 40cm
        "heavy_octo": 0.80,  # 80cm
    }

    def __init__(
        self,
        focal_length: float = 800.0,
        preset: str = "dji_mavic",
        ema_alpha: float = 0.30,
    ):
        self.focal_length = focal_length
        self.target_real_size = self.DEFAULT_PRESETS.get(preset, 0.40)
        self.ema_alpha = ema_alpha
        self.smoothed_dist: Optional[float] = None

    def calculate_optical_distance(self, bbox: Tuple[int, int, int, int]) -> float:
        """Pinhole formula: distance = (real_size * focal_length) / width_pixels."""
        x1, y1, x2, y2 = bbox
        width_px = max(1.0, float(x2 - x1))
        dist_m = (self.target_real_size * self.focal_length) / width_px
        return float(dist_m)

    def get_distance(
        self,
        bbox: Tuple[int, int, int, int],
        frame_shape: Tuple[int, int] = (480, 640),
        lidar_dist: Optional[float] = None,
    ) -> Tuple[float, str]:
        """Returns (distance_meters, source_name: 'lidar' | 'optical')."""
        if lidar_dist is not None and lidar_dist > 0.1:
            raw_dist = lidar_dist
            source = "lidar"
        else:
            raw_dist = self.calculate_optical_distance(bbox)
            source = "optical"

        # Apply EMA smoothing
        if self.smoothed_dist is None:
            self.smoothed_dist = raw_dist
        else:
            self.smoothed_dist = (
                self.ema_alpha * raw_dist + (1.0 - self.ema_alpha) * self.smoothed_dist
            )

        return float(self.smoothed_dist), source


def test_bbox_distance_formula():
    """T1.7.1: Verifies D = (W_real * f) / w_pixel produces exact expected distance."""
    estimator = DistanceEstimator(focal_length=800.0, preset="dji_mavic")
    # Real size = 0.40m, f = 800, bbox width = 32px -> D = (0.40 * 800) / 32 = 10.0m
    bbox = (100, 100, 132, 132)
    dist = estimator.calculate_optical_distance(bbox)
    assert dist == pytest.approx(10.0)


def test_bbox_distance_presets():
    """T1.7.2: Verifies distance calculations across different drone size presets."""
    est_mini = DistanceEstimator(focal_length=800.0, preset="dji_mini")
    est_heavy = DistanceEstimator(focal_length=800.0, preset="heavy_octo")

    bbox = (200, 200, 240, 240)  # width = 40px
    # Mini (0.25m): (0.25 * 800) / 40 = 5.0m
    assert est_mini.calculate_optical_distance(bbox) == pytest.approx(5.0)
    # Heavy (0.80m): (0.80 * 800) / 40 = 16.0m
    assert est_heavy.calculate_optical_distance(bbox) == pytest.approx(16.0)


def test_distance_estimator_lidar_priority():
    """T1.7.3: Verifies LiDAR is prioritized when present, falling back to optical when absent."""
    estimator = DistanceEstimator(focal_length=800.0, ema_alpha=1.0)
    bbox = (100, 100, 132, 132)  # Optical = 10.0m

    # When LiDAR reading is 12.5m, returns LiDAR
    dist, source = estimator.get_distance(bbox, lidar_dist=12.5)
    assert dist == pytest.approx(12.5)
    assert source == "lidar"

    # When LiDAR reading is None, returns Optical
    dist_opt, source_opt = estimator.get_distance(bbox, lidar_dist=None)
    assert dist_opt == pytest.approx(10.0)
    assert source_opt == "optical"


def test_distance_estimator_ema_smoothing():
    """T1.7.4: Verifies EMA smoothing dampens noise without step spikes."""
    estimator = DistanceEstimator(focal_length=800.0, ema_alpha=0.30)
    bbox_10m = (100, 100, 132, 132)  # 10m
    bbox_20m = (100, 100, 116, 116)  # 20m

    # Step 1: Initialized at 10m
    dist1, _ = estimator.get_distance(bbox_10m)
    assert dist1 == pytest.approx(10.0)

    # Step 2: Sudden jump to 20m -> smoothed = 0.3*20 + 0.7*10 = 13.0m
    dist2, _ = estimator.get_distance(bbox_20m)
    assert dist2 == pytest.approx(13.0)


def test_bbox_zero_width_safety():
    """T1.7.5: Verifies degenerate 0-width bbox is clamped to avoid ZeroDivisionError."""
    estimator = DistanceEstimator(focal_length=800.0)
    zero_box = (100, 100, 100, 100)
    dist = estimator.calculate_optical_distance(zero_box)
    assert dist > 0.0
    assert not math.isnan(dist)
    assert not math.isinf(dist)


def test_bbox_distance_scaling_with_range(video_gen):
    """T1.7.6: Verifies projected pixel width scales inversely with 3D range z."""
    t1 = TargetKinematics(x=0.0, y=0.0, z=20.0, real_size=0.40)
    t2 = TargetKinematics(x=0.0, y=0.0, z=40.0, real_size=0.40)

    box1 = t1.project_to_camera(video_gen.focal_length, video_gen.cx, video_gen.cy)
    box2 = t2.project_to_camera(video_gen.focal_length, video_gen.cx, video_gen.cy)

    w1 = box1[2] - box1[0]
    w2 = box2[2] - box2[0]

    # At 2x distance, width in pixels should be approximately half
    assert w1 == pytest.approx(2.0 * w2, abs=2.0)
