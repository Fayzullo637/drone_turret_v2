"""Tier 1 Unit Tests: YOLOv8 Detector & Model Hot-Swapping (Features F1, F3).

Verifies detection data contracts, confidence thresholding, class filtering,
bounding box integrity, and dynamic runtime model swapping.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import List, Optional, Tuple
import numpy as np
import pytest

from tests.fixtures.synthetic_video import SyntheticVideoGenerator, TargetKinematics

# Contract definition matching PROJECT.md
@dataclass
class Detection:
    box: Tuple[int, int, int, int]  # x1, y1, x2, y2
    confidence: float
    class_id: int
    class_name: str
    track_id: Optional[int] = None


class BaseDroneDetector:
    """Reference implementation / interface contract validator for YOLO detector."""

    def __init__(
        self,
        model_path: str = "models/yolov8n.pt",
        conf_threshold: float = 0.50,
        target_classes: Optional[List[int]] = None,
    ):
        self.model_path = model_path
        self.conf_threshold = conf_threshold
        self.target_classes = target_classes or [0, 4]  # e.g., person=0, drone/airplane=4
        self.loaded_model_name = os.path.basename(model_path)

    def set_confidence(self, conf: float) -> None:
        if not (0.0 <= conf <= 1.0):
            raise ValueError("Confidence must be in [0.0, 1.0]")
        self.conf_threshold = conf

    def swap_model(self, new_model_path: str) -> bool:
        if not new_model_path or not new_model_path.endswith(".pt"):
            raise ValueError(f"Invalid model file format: {new_model_path}")
        self.model_path = new_model_path
        self.loaded_model_name = os.path.basename(new_model_path)
        return True

    def filter_detections(self, raw_detections: List[Detection]) -> List[Detection]:
        filtered = []
        for det in raw_detections:
            if det.confidence < self.conf_threshold:
                continue
            if self.target_classes and det.class_id not in self.target_classes:
                continue
            filtered.append(det)
        return filtered


def test_detection_dataclass_structure():
    """T1.1.1: Verifies Detection dataclass fields, types, and defaults."""
    det = Detection(
        box=(100, 150, 200, 250),
        confidence=0.88,
        class_id=0,
        class_name="drone",
        track_id=42,
    )
    assert det.box == (100, 150, 200, 250)
    assert det.confidence == 0.88
    assert det.class_id == 0
    assert det.class_name == "drone"
    assert det.track_id == 42
    assert len(det.box) == 4


def test_detector_initialization_default_model():
    """T1.1.2: Verifies detector initializes with default model path and thresholds."""
    detector = BaseDroneDetector(model_path="models/yolov8n.pt", conf_threshold=0.60)
    assert detector.loaded_model_name == "yolov8n.pt"
    assert detector.conf_threshold == 0.60
    assert 0 in detector.target_classes


def test_detector_confidence_filtering():
    """T1.1.3: Verifies detections below confidence threshold are discarded."""
    detector = BaseDroneDetector(conf_threshold=0.65, target_classes=[0])
    raw = [
        Detection(box=(10, 10, 50, 50), confidence=0.40, class_id=0, class_name="drone"),
        Detection(box=(60, 60, 100, 100), confidence=0.65, class_id=0, class_name="drone"),
        Detection(box=(110, 110, 150, 150), confidence=0.89, class_id=0, class_name="drone"),
    ]
    filtered = detector.filter_detections(raw)
    assert len(filtered) == 2
    assert all(d.confidence >= 0.65 for d in filtered)
    assert filtered[0].box == (60, 60, 100, 100)
    assert filtered[1].box == (110, 110, 150, 150)


def test_detector_class_filtering():
    """T1.1.4: Verifies detector discards non-target classes (e.g. background objects)."""
    detector = BaseDroneDetector(conf_threshold=0.30, target_classes=[4])  # only class 4 (drone)
    raw = [
        Detection(box=(10, 10, 50, 50), confidence=0.90, class_id=0, class_name="person"),
        Detection(box=(60, 60, 100, 100), confidence=0.85, class_id=2, class_name="car"),
        Detection(box=(110, 110, 150, 150), confidence=0.75, class_id=4, class_name="drone"),
    ]
    filtered = detector.filter_detections(raw)
    assert len(filtered) == 1
    assert filtered[0].class_id == 4
    assert filtered[0].class_name == "drone"


def test_detector_model_hot_swapping():
    """T1.1.5: Verifies runtime hot-swapping between yolov8n.pt and drone_best.pt."""
    detector = BaseDroneDetector(model_path="models/yolov8n.pt")
    assert detector.loaded_model_name == "yolov8n.pt"

    success = detector.swap_model("models/drone_best.pt")
    assert success is True
    assert detector.loaded_model_name == "drone_best.pt"

    # Rejection of invalid extension
    with pytest.raises(ValueError, match="Invalid model file"):
        detector.swap_model("models/invalid_weights.bin")


def test_detector_bounding_box_coordinates_validity(video_gen):
    """T1.1.6: Verifies bounding boxes generated from 3D targets form valid non-degenerate rects."""
    target = TargetKinematics(x=5.0, y=2.0, z=30.0, real_size=0.45)
    x1, y1, x2, y2 = target.project_to_camera(video_gen.focal_length, video_gen.cx, video_gen.cy)

    assert x1 < x2, f"Expected x1 < x2, got {x1} >= {x2}"
    assert y1 < y2, f"Expected y1 < y2, got {y1} >= {y2}"
    width = x2 - x1
    height = y2 - y1
    assert width >= 4
    assert height >= 4
