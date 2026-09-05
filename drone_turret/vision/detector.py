"""YOLOv8 Object Detection and ByteTrack Multi-Object Tracking Module.

Features:
- YOLOv8 inference wrapper with persistent ByteTrack tracking.
- Dynamic runtime model hot-swapping without restarting the application.
- Target class filtering (e.g. drone-specific vs COCO classes) and confidence thresholding.
- Single-target lock retention logic (locked_track_id prioritization and area-based auto-lock).
- Structured Detection and DetectionResult dataclasses.
- Tactical frame annotation utilities.
"""

from __future__ import annotations

import logging
import os
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

import cv2
import numpy as np
from ultralytics import YOLO

logger = logging.getLogger(__name__)


@dataclass
class Detection:
    """Represents a single detected and tracked object."""
    box: Tuple[int, int, int, int]  # (x1, y1, x2, y2) in pixel coordinates
    confidence: float
    class_id: int
    class_name: str
    track_id: Optional[int] = None

    @property
    def x1(self) -> int:
        """Left coordinate."""
        return self.box[0]

    @property
    def y1(self) -> int:
        """Top coordinate."""
        return self.box[1]

    @property
    def x2(self) -> int:
        """Right coordinate."""
        return self.box[2]

    @property
    def y2(self) -> int:
        """Bottom coordinate."""
        return self.box[3]

    @property
    def width(self) -> int:
        """Bounding box width in pixels."""
        return max(0, self.box[2] - self.box[0])

    @property
    def height(self) -> int:
        """Bounding box height in pixels."""
        return max(0, self.box[3] - self.box[1])

    @property
    def center(self) -> Tuple[int, int]:
        """Centroid pixel coordinate (cx, cy)."""
        return ((self.box[0] + self.box[2]) // 2, (self.box[1] + self.box[3]) // 2)

    @property
    def cx(self) -> int:
        """Centroid X in pixels."""
        return (self.box[0] + self.box[2]) // 2

    @property
    def cy(self) -> int:
        """Centroid Y in pixels."""
        return (self.box[1] + self.box[3]) // 2

    @property
    def area(self) -> int:
        """Bounding box area in square pixels."""
        return self.width * self.height

    def to_dict(self) -> Dict[str, Any]:
        """Convert detection to JSON-serializable dictionary."""
        return {
            "box": list(self.box),
            "confidence": round(float(self.confidence), 4),
            "class_id": int(self.class_id),
            "class_name": str(self.class_name),
            "track_id": int(self.track_id) if self.track_id is not None else None,
            "center": [self.cx, self.cy],
            "width": self.width,
            "height": self.height,
            "area": self.area,
        }


@dataclass
class DetectionResult:
    """Output container from detector inference and tracking."""
    detections: List[Detection]
    locked_target: Optional[Detection]
    target_locked: bool
    locked_track_id: Optional[int]
    inference_time_ms: float
    model_name: str
    frame_shape: Tuple[int, int]  # (height, width)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "detections": [d.to_dict() for d in self.detections],
            "locked_target": self.locked_target.to_dict() if self.locked_target else None,
            "target_locked": self.target_locked,
            "locked_track_id": self.locked_track_id,
            "inference_time_ms": round(self.inference_time_ms, 2),
            "model_name": self.model_name,
            "frame_shape": list(self.frame_shape),
        }


class YOLODetector:
    """YOLOv8 Object Detector with ByteTrack Tracking and Dynamic Hot-Swapping."""

    DEFAULT_MODEL_SEARCH_DIRS = [
        Path("models"),
        Path("drone_turret_v2/models"),
        Path("../models"),
        Path("."),
    ]

    def __init__(
        self,
        model_path: str = "models/drone_best.pt",
        conf_threshold: float = 0.45,
        target_classes: Optional[List[int]] = None,
        tracker: str = "bytetrack.yaml",
    ):
        self.conf_threshold = float(conf_threshold)
        self.target_classes = target_classes  # e.g. [0, 1, 2, 3, 4] or None for all
        self.tracker = tracker

        self._lock = threading.Lock()
        self.model_path = model_path
        self.model_name = os.path.basename(model_path)
        self.model: Optional[YOLO] = None
        self.class_names: Dict[int, str] = {}

        # Target lock management
        self.locked_track_id: Optional[int] = None
        self.auto_lock: bool = True

        # Load initial model
        self._load_model(model_path)

    def _resolve_model_path(self, path_str: str) -> Path:
        """Locate model file checking relative and standard model directories."""
        candidate = Path(path_str)
        if candidate.exists() and candidate.is_file():
            return candidate

        # Check search paths
        for search_dir in self.DEFAULT_MODEL_SEARCH_DIRS:
            test_path = search_dir / candidate.name
            if test_path.exists() and test_path.is_file():
                return test_path

        # If not found, return original path and let YOLO attempt download or raise
        return candidate

    def _load_model(self, path_str: str) -> None:
        """Internal model loader with class names extraction."""
        resolved = self._resolve_model_path(path_str)
        logger.info("Loading YOLO model from: %s", resolved)

        model = YOLO(str(resolved))
        # Extract class mapping
        names = getattr(model, "names", {})
        if isinstance(names, dict):
            self.class_names = {int(k): str(v) for k, v in names.items()}
        elif isinstance(names, list):
            self.class_names = {i: str(n) for i, n in enumerate(names)}
        else:
            self.class_names = {}

        self.model = model
        self.model_path = str(resolved)
        self.model_name = os.path.basename(str(resolved))
        logger.info(
            "Model '%s' loaded successfully with %d classes.",
            self.model_name,
            len(self.class_names),
        )

    def switch_model(self, model_path: str, reset_lock: bool = False) -> bool:
        """Dynamically switch YOLO model weights at runtime (Thread-safe)."""
        with self._lock:
            try:
                resolved = self._resolve_model_path(model_path)
                new_model = YOLO(str(resolved))
                names = getattr(new_model, "names", {})
                if isinstance(names, dict):
                    new_names = {int(k): str(v) for k, v in names.items()}
                elif isinstance(names, list):
                    new_names = {i: str(n) for i, n in enumerate(names)}
                else:
                    new_names = {}

                self.model = new_model
                self.model_path = str(resolved)
                self.model_name = os.path.basename(str(resolved))
                self.class_names = new_names

                if reset_lock:
                    self.locked_track_id = None

                logger.info("Hot-swapped model to: %s", self.model_name)
                return True
            except Exception as err:
                logger.error("Failed to hot-swap model to '%s': %s", model_path, err)
                return False

    def lock_track(self, track_id: int) -> None:
        """Explicitly lock onto a specific track ID."""
        with self._lock:
            self.locked_track_id = int(track_id)
            logger.info("Manually locked onto track ID: %d", self.locked_track_id)

    def unlock_track(self) -> None:
        """Release current target lock."""
        with self._lock:
            self.locked_track_id = None
            logger.info("Target lock released.")

    def reset_lock(self) -> None:
        """Alias for unlock_track."""
        self.unlock_track()

    def set_target_classes(self, classes: Optional[List[int]]) -> None:
        """Configure allowed class IDs for detection filter."""
        with self._lock:
            self.target_classes = [int(c) for c in classes] if classes is not None else None

    def set_confidence_threshold(self, conf: float) -> None:
        """Configure minimum detection confidence threshold."""
        with self._lock:
            self.conf_threshold = max(0.01, min(0.99, float(conf)))

    def get_class_names(self) -> Dict[int, str]:
        """Return dictionary of class IDs and names for the active model."""
        with self._lock:
            return dict(self.class_names)

    def get_model_info(self) -> Dict[str, Any]:
        """Return current model metadata and configuration."""
        with self._lock:
            return {
                "model_name": self.model_name,
                "model_path": self.model_path,
                "conf_threshold": self.conf_threshold,
                "target_classes": self.target_classes,
                "tracker": self.tracker,
                "locked_track_id": self.locked_track_id,
                "class_count": len(self.class_names),
                "classes": self.class_names,
            }

    def detect(
        self,
        frame: np.ndarray,
        conf: Optional[float] = None,
        target_classes: Optional[List[int]] = None,
        persist: bool = True,
    ) -> DetectionResult:
        """Run YOLOv8 inference with ByteTrack multi-object tracking.
        
        Args:
            frame: Input BGR image frame (H x W x 3).
            conf: Override confidence threshold if provided.
            target_classes: Override target class filter list if provided.
            persist: Keep track state across sequential frames (True for video).
        """
        if frame is None or not isinstance(frame, np.ndarray) or frame.size == 0:
            return DetectionResult(
                detections=[],
                locked_target=None,
                target_locked=False,
                locked_track_id=self.locked_track_id,
                inference_time_ms=0.0,
                model_name=self.model_name,
                frame_shape=(0, 0),
            )

        h, w = frame.shape[:2]
        effective_conf = float(conf) if conf is not None else self.conf_threshold
        effective_classes = target_classes if target_classes is not None else self.target_classes

        start_t = time.perf_counter()

        with self._lock:
            if self.model is None:
                return DetectionResult(
                    detections=[],
                    locked_target=None,
                    target_locked=False,
                    locked_track_id=None,
                    inference_time_ms=0.0,
                    model_name="",
                    frame_shape=(h, w),
                )

            try:
                # Run YOLO tracking with ByteTrack
                results = self.model.track(
                    frame,
                    conf=effective_conf,
                    persist=persist,
                    tracker=self.tracker,
                    verbose=False,
                )
            except Exception as err:
                logger.warning("Error during YOLO tracking, trying predict fallback: %s", err)
                try:
                    results = self.model.predict(
                        frame,
                        conf=effective_conf,
                        verbose=False,
                    )
                except Exception as fatal_err:
                    logger.error("YOLO inference failed: %s", fatal_err)
                    return DetectionResult(
                        detections=[],
                        locked_target=None,
                        target_locked=False,
                        locked_track_id=self.locked_track_id,
                        inference_time_ms=0.0,
                        model_name=self.model_name,
                        frame_shape=(h, w),
                    )

        inference_time_ms = (time.perf_counter() - start_t) * 1000.0

        detections: List[Detection] = []

        if results and len(results) > 0:
            res = results[0]
            boxes = getattr(res, "boxes", None)
            if boxes is not None and len(boxes) > 0:
                for box in boxes:
                    # Confidence & Class ID
                    box_conf = float(box.conf[0].cpu().item()) if box.conf is not None else 0.0
                    box_cls = int(box.cls[0].cpu().item()) if box.cls is not None else 0

                    # Filter by confidence
                    if box_conf < effective_conf:
                        continue

                    # Filter by target class
                    if effective_classes is not None and box_cls not in effective_classes:
                        continue

                    # Coordinates
                    xyxy = box.xyxy[0].cpu().numpy().astype(int)
                    x1 = max(0, min(w - 1, int(xyxy[0])))
                    y1 = max(0, min(h - 1, int(xyxy[1])))
                    x2 = max(x1 + 1, min(w, int(xyxy[2])))
                    y2 = max(y1 + 1, min(h, int(xyxy[3])))

                    # Track ID
                    track_id: Optional[int] = None
                    if hasattr(box, "id") and box.id is not None:
                        try:
                            track_id = int(box.id[0].cpu().item())
                        except Exception:
                            track_id = None

                    cls_name = self.class_names.get(box_cls, f"class_{box_cls}")

                    detections.append(
                        Detection(
                            box=(x1, y1, x2, y2),
                            confidence=box_conf,
                            class_id=box_cls,
                            class_name=cls_name,
                            track_id=track_id,
                        )
                    )

        # Single-target locking & prioritization logic
        locked_target: Optional[Detection] = None
        target_locked = False

        with self._lock:
            current_locked_id = self.locked_track_id

            if current_locked_id is not None:
                # Search for designated track ID in current detections
                for d in detections:
                    if d.track_id is not None and d.track_id == current_locked_id:
                        locked_target = d
                        target_locked = True
                        break

            # If no target locked yet or designated target lost, select best candidate
            if locked_target is None and len(detections) > 0:
                if self.auto_lock or current_locked_id is None:
                    # Select largest bounding box area (closest drone)
                    best_det = max(detections, key=lambda d: (d.area, d.confidence))
                    locked_target = best_det
                    target_locked = True
                    if best_det.track_id is not None:
                        self.locked_track_id = best_det.track_id
                        current_locked_id = best_det.track_id

        return DetectionResult(
            detections=detections,
            locked_target=locked_target,
            target_locked=target_locked,
            locked_track_id=self.locked_track_id,
            inference_time_ms=inference_time_ms,
            model_name=self.model_name,
            frame_shape=(h, w),
        )

    # Alias for method compatibility
    detect_and_track = detect

    def draw_detections(
        self,
        frame: np.ndarray,
        result: DetectionResult,
        draw_reticle: bool = True,
        locked_color: Tuple[int, int, int] = (0, 0, 255),      # Red for locked target
        unlocked_color: Tuple[int, int, int] = (0, 255, 0),    # Green for other tracks
    ) -> np.ndarray:
        """Render tactical bounding boxes and reticles on a frame copy."""
        annotated = frame.copy()
        h, w = annotated.shape[:2]
        cx_screen = w // 2
        cy_screen = h // 2

        # Draw central screen reticle
        if draw_reticle:
            reticle_color = (255, 128, 0)  # Cyan/Orange
            cv2.line(annotated, (cx_screen - 15, cy_screen), (cx_screen + 15, cy_screen), reticle_color, 1)
            cv2.line(annotated, (cx_screen, cy_screen - 15), (cx_screen, cy_screen + 15), reticle_color, 1)
            cv2.circle(annotated, (cx_screen, cy_screen), 24, reticle_color, 1)

        # Draw all detections
        for det in result.detections:
            is_locked = (
                result.locked_target is not None
                and result.locked_target.track_id is not None
                and det.track_id == result.locked_target.track_id
            ) or (det == result.locked_target)

            color = locked_color if is_locked else unlocked_color
            thickness = 2 if is_locked else 1

            # Bounding box
            cv2.rectangle(annotated, (det.x1, det.y1), (det.x2, det.y2), color, thickness)

            # Center point
            cv2.circle(annotated, (det.cx, det.cy), 4, color, -1)

            # Label text
            tid_str = f"#{det.track_id} " if det.track_id is not None else ""
            label = f"{tid_str}{det.class_name} {det.confidence:.0%}"
            
            # Label background banner
            (lbl_w, lbl_h), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
            cv2.rectangle(
                annotated,
                (det.x1, max(0, det.y1 - lbl_h - 6)),
                (det.x1 + lbl_w + 4, det.y1),
                color,
                -1,
            )
            cv2.putText(
                annotated,
                label,
                (det.x1 + 2, max(lbl_h + 2, det.y1 - 4)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 255) if is_locked else (0, 0, 0),
                1,
                cv2.LINE_AA,
            )

            # If locked, draw line from screen center to target
            if is_locked:
                cv2.line(annotated, (cx_screen, cy_screen), (det.cx, det.cy), (0, 255, 255), 1, cv2.LINE_AA)

        return annotated


# Type alias for interoperability
YOLOTrackerDetector = YOLODetector
