"""
Tier 1 Feature Tests: F13 - Passive Optical Bounding Box Ranging & Sensor Fusion.
"""

from __future__ import annotations

import math
import pytest

from drone_turret.sensors.distance import (
    DRONE_PRESETS,
    DistanceEstimator,
    compute_focal_length_px,
    compute_hfov_deg,
    get_drone_size,
)


def test_focal_length_and_hfov_reciprocity():
    """Verifies HFOV <-> focal length mathematical reciprocity."""
    width = 640
    hfov_target = 70.0
    
    f_px = compute_focal_length_px(width, hfov_target)
    # For 640px @ 70°: f = (320) / tan(35°) ≈ 320 / 0.7002075 ≈ 457.01 px
    assert pytest.approx(f_px, abs=0.1) == 457.01
    
    recovered_hfov = compute_hfov_deg(width, f_px)
    assert pytest.approx(recovered_hfov, abs=0.01) == 70.0


def test_drone_presets_resolution():
    """Verifies lookup of drone wingspan presets."""
    assert get_drone_size("DJI Mavic 3") == 0.35
    assert get_drone_size("Shahed-136") == 2.50
    assert get_drone_size("FPV") == 0.22
    assert get_drone_size("MQ-9") == 20.0
    assert get_drone_size("Unknown", custom_size_m=1.23) == 1.23


def test_optical_pinhole_distance_formula():
    """Verifies pinhole distance formula: distance = (size * focal_length) / bbox_width."""
    estimator = DistanceEstimator(default_preset="DJI Mavic 3", camera_hfov_deg=70.0)
    
    # 640x480 frame -> f_px ≈ 457.01
    # Target size = 0.35m
    # If bbox width is 45.7 pixels, distance should be exactly 3.50 meters
    frame_shape = (480, 640)
    bbox = (100, 100, 145.7, 130)  # width = 45.7 px
    
    dist = estimator.estimate_optical_distance(bbox, frame_shape, preset="DJI Mavic 3")
    assert pytest.approx(dist, abs=0.05) == 3.50


def test_distance_fusion_lidar_primary():
    """Verifies LiDAR is selected as primary source when valid."""
    estimator = DistanceEstimator()
    frame_shape = (480, 640)
    bbox = (200, 150, 250, 190)

    # Valid LiDAR reading of 14.2 meters
    dist, source = estimator.get_distance(
        bbox=bbox,
        frame_shape=frame_shape,
        lidar_dist=14.2,
    )
    assert dist == 14.2
    assert source == "lidar"


def test_distance_fusion_optical_fallback():
    """Verifies seamless fallback to optical ranging when LiDAR is absent or None."""
    estimator = DistanceEstimator(default_preset="Shahed-136", camera_hfov_deg=70.0)
    frame_shape = (480, 640)
    bbox = (200, 150, 300, 200)  # width = 100 px

    # Case 1: LiDAR is None
    dist1, source1 = estimator.get_distance(
        bbox=bbox,
        frame_shape=frame_shape,
        lidar_dist=None,
    )
    assert source1 == "optical"
    assert dist1 > 0

    # Case 2: LiDAR is invalid (e.g. 0.0 or negative)
    dist2, source2 = estimator.get_distance(
        bbox=bbox,
        frame_shape=frame_shape,
        lidar_dist=0.0,
    )
    assert source2 == "optical"
    assert dist2 == dist1


def test_3d_coordinate_and_speed_recovery():
    """Verifies 3D world coordinate transformation and kinematic speed in km/h."""
    estimator = DistanceEstimator(camera_hfov_deg=70.0)
    frame_shape = (480, 640)
    
    # Object centered on screen at 10.0m range
    pos_3d = estimator.calculate_3d_position(
        pixel_xy=(320.0, 240.0),
        distance_m=10.0,
        frame_shape=frame_shape,
    )
    assert pos_3d == (0.0, 0.0, 10.0)

    # Object moving horizontally at 100 px/s at 10.0m range
    # f_px ≈ 457.01 -> vx = (100 * 10) / 457.01 ≈ 2.188 m/s -> speed_kmh ≈ 2.188 * 3.6 ≈ 7.88 km/h
    speed_kmh = estimator.calculate_speed_kmh(
        pixel_vel_xy=(100.0, 0.0),
        distance_m=10.0,
        frame_shape=frame_shape,
    )
    assert pytest.approx(speed_kmh, abs=0.1) == 7.88
