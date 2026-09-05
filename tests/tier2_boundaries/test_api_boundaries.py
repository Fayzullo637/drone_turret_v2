"""Tier 2 Boundary Tests: FastAPI Schema & Request Validation Boundaries (Features F16, F18).

Tests REST API payload validation boundaries: out-of-bounds confidence thresholds,
zero/negative projectile physics params, malformed JSON, and unknown route errors.
"""

from __future__ import annotations

import pytest
from fastapi.testclient import TestClient

from tests.tier1_features.test_fastapi_routes import create_test_fastapi_app


@pytest.fixture
def client() -> TestClient:
    app = create_test_fastapi_app()
    return TestClient(app)


def test_api_negative_confidence(client):
    """T2.5.1: Verifies POST /api/config with confidence < 0 returns HTTP 422."""
    res = client.post("/api/config", json={"confidence": -0.25})
    assert res.status_code == 422


def test_api_confidence_gt_one(client):
    """T2.5.2: Verifies POST /api/config with confidence > 1.0 returns HTTP 422."""
    res = client.post("/api/config", json={"confidence": 1.25})
    assert res.status_code == 422


def test_api_zero_muzzle_velocity(client):
    """T2.5.3: Verifies POST /api/config with muzzle_velocity <= 0 returns HTTP 422."""
    res = client.post("/api/config", json={"muzzle_velocity": 0.0})
    assert res.status_code == 422


def test_api_negative_net_mass(client):
    """T2.5.4: Verifies POST /api/config with negative net mass returns HTTP 422."""
    res = client.post("/api/config", json={"net_mass": -0.5})
    assert res.status_code == 422


def test_api_extreme_drag_cd(client):
    """T2.5.5: Verifies POST /api/config with net_cd out of valid range returns HTTP 422."""
    res = client.post("/api/config", json={"net_cd": 5.0})
    assert res.status_code == 422


def test_api_unknown_endpoint_404(client):
    """T2.5.6: Verifies requesting non-existent endpoint returns HTTP 404."""
    res = client.get("/api/unknown_route")
    assert res.status_code == 404
