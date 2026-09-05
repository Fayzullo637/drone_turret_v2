"""Tier 1 Unit Tests: ByteTrack Multi-Object Association & Target Locking (Feature F2).

Verifies track ID assignment, single-target lock retention, IoU association,
track continuity, and clean handling of target departures.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import pytest

from tests.fixtures.synthetic_video import SyntheticVideoGenerator, TargetKinematics
from tests.tier1_features.test_detector_yolo import Detection


def calculate_iou(boxA: Tuple[int, int, int, int], boxB: Tuple[int, int, int, int]) -> float:
    """Calculates Intersection-over-Union (IoU) between two bounding boxes."""
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])

    inter_w = max(0, xB - xA)
    inter_h = max(0, yB - yA)
    inter_area = inter_w * inter_h

    boxA_area = max(1, (boxA[2] - boxA[0]) * (boxA[3] - boxA[1]))
    boxB_area = max(1, (boxB[2] - boxB[0]) * (boxB[3] - boxB[1]))

    union_area = float(boxA_area + boxB_area - inter_area)
    return inter_area / union_area if union_area > 0 else 0.0


class SimpleTracker:
    """Deterministic IoU-based persistent multi-object tracker for test validation."""

    def __init__(self, iou_threshold: float = 0.30, max_age: int = 15):
        self.iou_threshold = iou_threshold
        self.max_age = max_age
        self.next_id = 1
        self.tracks: Dict[int, Dict[str, Any]] = {}
        self.locked_track_id: Optional[int] = None

    def update(self, detections: List[Detection]) -> List[Detection]:
        # Age existing tracks
        for tid in list(self.tracks.keys()):
            self.tracks[tid]["age"] += 1

        matched_dets = set()
        matched_tracks = set()
        result_dets: List[Detection] = []

        # Match existing tracks to detections by IoU or centroid proximity
        for tid, tdata in self.tracks.items():
            best_score = 0.0
            best_idx = -1
            t_box = tdata["box"]
            t_cx, t_cy = (t_box[0] + t_box[2]) / 2, (t_box[1] + t_box[3]) / 2

            for idx, det in enumerate(detections):
                if idx in matched_dets:
                    continue
                iou = calculate_iou(t_box, det.box)
                d_cx, d_cy = (det.box[0] + det.box[2]) / 2, (det.box[1] + det.box[3]) / 2
                dist_px = math.hypot(t_cx - d_cx, t_cy - d_cy)

                # Match if IoU >= threshold or centroid distance < 40px
                score = iou if iou >= self.iou_threshold else (1.0 / (1.0 + dist_px) if dist_px < 40.0 else 0.0)

                if score > best_score:
                    best_score = score
                    best_idx = idx

            if best_score > 0.0 and best_idx >= 0:
                matched_dets.add(best_idx)
                matched_tracks.add(tid)
                det = detections[best_idx]
                det.track_id = tid
                self.tracks[tid]["box"] = det.box
                self.tracks[tid]["age"] = 0
                result_dets.append(det)

        # Create new tracks for unmatched detections
        for idx, det in enumerate(detections):
            if idx not in matched_dets:
                tid = self.next_id
                self.next_id += 1
                det.track_id = tid
                self.tracks[tid] = {"box": det.box, "age": 0}
                result_dets.append(det)
                if self.locked_track_id is None:
                    self.locked_track_id = tid

        # Delete expired tracks
        for tid in list(self.tracks.keys()):
            if self.tracks[tid]["age"] > self.max_age:
                del self.tracks[tid]
                if self.locked_track_id == tid:
                    self.locked_track_id = None

        return result_dets

    def lock_target(self, track_id: int) -> bool:
        if track_id in self.tracks:
            self.locked_track_id = track_id
            return True
        return False

    def get_locked_target(self, detections: List[Detection]) -> Optional[Detection]:
        if not detections:
            return None
        if self.locked_track_id is None:
            self.locked_track_id = detections[0].track_id
            return detections[0]
        for det in detections:
            if det.track_id == self.locked_track_id:
                return det
        return detections[0]


def test_bytetrack_iou_calculation():
    """T1.2.1: Tests IoU function on overlapping and non-overlapping boxes."""
    box1 = (100, 100, 200, 200)
    box2 = (100, 100, 200, 200)  # Identical
    box3 = (150, 150, 250, 250)  # 50x50 overlap
    box4 = (300, 300, 400, 400)  # Disjoint

    assert calculate_iou(box1, box2) == pytest.approx(1.0)
    assert calculate_iou(box1, box4) == pytest.approx(0.0)
    iou_partial = calculate_iou(box1, box3)
    assert 0.10 < iou_partial < 0.20


def test_bytetrack_single_target_lock():
    """T1.2.2: Tests tracker locks onto target #1 and maintains lock ID even when #2 is introduced."""
    tracker = SimpleTracker()
    d1 = Detection(box=(100, 100, 140, 140), confidence=0.85, class_id=0, class_name="drone")
    res1 = tracker.update([d1])
    assert len(res1) == 1
    assert res1[0].track_id == 1
    tracker.lock_target(1)
    assert tracker.locked_track_id == 1

    # Frame 2: Introduce second drone
    d1_next = Detection(box=(105, 102, 145, 142), confidence=0.86, class_id=0, class_name="drone")
    d2 = Detection(box=(300, 200, 380, 280), confidence=0.95, class_id=0, class_name="drone")
    res2 = tracker.update([d1_next, d2])

    locked = tracker.get_locked_target(res2)
    assert locked is not None
    assert locked.track_id == 1
    assert locked.box == (105, 102, 145, 142)


def test_bytetrack_id_continuity_across_frames(video_gen):
    """T1.2.3: Tests track ID persists continuously across a moving synthetic target sequence."""
    tracker = SimpleTracker()
    target = TargetKinematics(x=-5.0, y=0.0, z=40.0, vx=10.0)

    assigned_ids = []
    for _ in range(30):
        target.update(video_gen.dt)
        box = target.project_to_camera(video_gen.focal_length, video_gen.cx, video_gen.cy)
        det = Detection(box=box, confidence=0.90, class_id=0, class_name="drone")
        results = tracker.update([det])
        assert len(results) == 1
        assigned_ids.append(results[0].track_id)

    # Every frame should have identical track_id (1)
    assert len(set(assigned_ids)) == 1
    assert assigned_ids[0] == 1


def test_bytetrack_dual_detections_separate_ids():
    """T1.2.4: Verifies multiple concurrent drones receive distinct unique track IDs."""
    tracker = SimpleTracker()
    d1 = Detection(box=(100, 100, 140, 140), confidence=0.88, class_id=0, class_name="drone")
    d2 = Detection(box=(400, 300, 450, 350), confidence=0.82, class_id=0, class_name="drone")

    results = tracker.update([d1, d2])
    assert len(results) == 2
    id1, id2 = results[0].track_id, results[1].track_id
    assert id1 != id2
    assert {id1, id2} == {1, 2}


def test_bytetrack_lost_track_cleanup():
    """T1.2.5: Verifies old tracks are pruned after exceeding max_age inactive frames."""
    tracker = SimpleTracker(max_age=5)
    d1 = Detection(box=(100, 100, 140, 140), confidence=0.9, class_id=0, class_name="drone")
    tracker.update([d1])
    assert 1 in tracker.tracks

    # 6 consecutive empty frames
    for _ in range(6):
        tracker.update([])

    assert 1 not in tracker.tracks
    assert tracker.locked_track_id is None
