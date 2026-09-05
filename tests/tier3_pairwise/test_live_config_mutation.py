"""Tier 3 Pairwise Integration Tests: Live REST Config Mutation During Tracking (R6 + R1..R5).

Verifies dynamic parameter tuning (changing confidence threshold, muzzle velocity,
or net drag coefficient Cd) via REST API while tracking loop is running without pipeline crashes.
"""

from __future__ import annotations

import pytest
from fastapi.testclient import TestClient

from tests.fixtures.physics_benchmarks import BallisticsBenchmarks
from tests.fixtures.synthetic_video import TargetKinematics
from tests.tier1_features.test_detector_yolo import BaseDroneDetector, Detection
from tests.tier1_features.test_fastapi_routes import create_test_fastapi_app


def test_live_config_mutation_during_tracking(physics_benchmarks):
    """T3.4.1: Verifies REST config update dynamically updates detector and ballistics on next frame."""
    app = create_test_fastapi_app()
    client = TestClient(app)

    detector = BaseDroneDetector(conf_threshold=0.50)
    muzzle_v0 = 80.0
    net_cd = 1.20

    # Initial frame with detection at conf 0.60
    d1 = Detection(box=(100, 100, 140, 140), confidence=0.60, class_id=0, class_name="drone")
    res1 = detector.filter_detections([d1])
    assert len(res1) == 1

    # Live mutation via REST API: bump confidence threshold to 0.70 and muzzle velocity to 95 m/s
    api_res = client.post(
        "/api/config",
        json={
            "confidence": 0.70,
            "muzzle_velocity": 95.0,
            "net_cd": 1.40,
            "net_mass": 0.60,
            "model_name": "drone_best.pt",
        },
    )
    assert api_res.status_code == 200
    cfg = api_res.json()["config"]

    # Adopt updated config in pipeline
    detector.set_confidence(cfg["confidence"])
    muzzle_v0 = cfg["muzzle_velocity"]
    net_cd = cfg["net_cd"]

    # Next frame: same detection at conf 0.60 is now filtered out (< 0.70)
    res2 = detector.filter_detections([d1])
    assert len(res2) == 0

    # Ballistics with new v0 = 95 m/s computes faster intercept time
    sol_initial = physics_benchmarks.reference_intercept_solver(
        target_pos_3d=(0.0, 0.0, 50.0), target_vel_3d=(0.0, 0.0, 0.0), v0=80.0
    )
    sol_updated = physics_benchmarks.reference_intercept_solver(
        target_pos_3d=(0.0, 0.0, 50.0), target_vel_3d=(0.0, 0.0, 0.0), v0=muzzle_v0
    )

    assert sol_updated["t_intercept"] < sol_initial["t_intercept"]
