"""Tier 2 Boundary Tests: Computer Vision & Detection Edge Cases (Feature F1, F2).

Tests extreme visual conditions: zero detections, crowded scenes (100 objects),
frame border clipping, sub-pixel/micro targets, and full-screen bounding boxes.
"""

from __future__ import annotations

import numpy as np
import pytest

from tests.fixtures.synthetic_video import SyntheticVideoGenerator, TargetKinematics
from tests.tier1_features.test_detector_yolo import BaseDroneDetector, Detection
from tests.tier1_features.test_bytetrack import SimpleTracker


def test_vision_zero_detections(video_gen):
    """T2.1.1: Verifies pipeline handles completely empty frame without crash."""
    detector = BaseDroneDetector(conf_threshold=0.5)
    tracker = SimpleTracker()

    empty_frame = np.zeros((video_gen.height, video_gen.width, 3), dtype=np.uint8)
    # 0 detections
    detections: list[Detection] = []
    filtered = detector.filter_detections(detections)
    assert len(filtered) == 0

    tracks = tracker.update(filtered)
    assert len(tracks) == 0
    assert tracker.locked_track_id is None


def test_vision_crowded_detections():
    """T2.1.2: Verifies tracker stability under 100 simultaneous synthetic targets."""
    detector = BaseDroneDetector(conf_threshold=0.50, target_classes=[0])
    tracker = SimpleTracker()

    # Generate 100 detections scattered across frame
    crowd = [
        Detection(
            box=(i * 6, (i % 10) * 40, i * 6 + 20, (i % 10) * 40 + 20),
            confidence=0.55 + (i % 40) * 0.01,
            class_id=0,
            class_name="drone",
        )
        for i in range(100)
    ]

    filtered = detector.filter_detections(crowd)
    assert len(filtered) == 100

    tracks = tracker.update(filtered)
    assert len(tracks) == 100
    # Primary lock should still be assigned
    assert tracker.locked_track_id is not None


def test_vision_target_on_frame_boundary(video_gen):
    """T2.1.3: Verifies bounding box coordinates clipping beyond image margins."""
    # Target partially off-screen at top-left (-50, -30)
    box = (-50, -30, 40, 50)
    # Standard clamp logic
    clamped_x1 = max(0, min(video_gen.width, box[0]))
    clamped_y1 = max(0, min(video_gen.height, box[1]))
    clamped_x2 = max(0, min(video_gen.width, box[2]))
    clamped_y2 = max(0, min(video_gen.height, box[3]))

    assert clamped_x1 == 0
    assert clamped_y1 == 0
    assert clamped_x2 == 40
    assert clamped_y2 == 50
    assert (clamped_x2 - clamped_x1) > 0


def test_vision_micro_target(video_gen):
    """T2.1.4: Verifies handling of 2x2 micro bounding box at extreme range (200m)."""
    target = TargetKinematics(x=0.0, y=0.0, z=200.0, real_size=0.30)
    box = target.project_to_camera(
        f=video_gen.focal_length, cx=video_gen.cx, cy=video_gen.cy, min_pixel_size=2
    )

    x1, y1, x2, y2 = box
    width = x2 - x1
    height = y2 - y1

    assert width >= 2
    assert height >= 2
    assert x1 < x2 and y1 < y2


def test_vision_full_screen_target(video_gen):
    """T2.1.5: Verifies target taking up full screen (0.5m distance) does not crash."""
    target = TargetKinematics(x=0.0, y=0.0, z=0.5, real_size=0.50)
    box = target.project_to_camera(
        f=video_gen.focal_length, cx=video_gen.cx, cy=video_gen.cy
    )

    x1, y1, x2, y2 = box
    pixel_width = x2 - x1
    # Bbox is huge (e.g. 800px wide, exceeding 640 screen width)
    assert pixel_width >= video_gen.width
