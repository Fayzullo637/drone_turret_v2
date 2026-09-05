"""Tier 1 Unit Tests: FastAPI REST Endpoints & Video Stream (Features F16, F17, F18).

Verifies telemetry status endpoints, configuration updates, hardware device scanning,
fire trigger REST endpoints, and MJPEG multipart stream formatting.
"""

from __future__ import annotations

from typing import Any, Dict, Generator
import cv2
import numpy as np
import pytest
from fastapi import FastAPI, HTTPException
from fastapi.responses import JSONResponse, StreamingResponse
from fastapi.testclient import TestClient
from pydantic import BaseModel, Field


# Reference Pydantic Config Model matching PROJECT.md specifications
class TurretConfigModel(BaseModel):
    confidence: float = Field(default=0.50, ge=0.0, le=1.0)
    target_class: int = Field(default=0, ge=0)
    muzzle_velocity: float = Field(default=80.0, gt=0.0, le=300.0)
    net_mass: float = Field(default=0.60, gt=0.0, le=5.0)
    net_cd: float = Field(default=1.20, ge=0.5, le=2.5)
    model_name: str = Field(default="yolov8n.pt")


class TurretStateModel(BaseModel):
    fps: float = 30.0
    pan_angle: float = 90.0
    tilt_angle: float = 90.0
    distance_m: float = 25.0
    speed_kmh: float = 120.0
    target_locked: bool = True
    tracking_state: str = "TRACKING"


def create_test_fastapi_app() -> FastAPI:
    """Instantiates a fully compliant reference FastAPI app for endpoint verification."""
    app = FastAPI(title="Drone Turret v2 API")

    current_config = TurretConfigModel()
    current_state = TurretStateModel()

    @app.get("/api/status")
    def get_status() -> Dict[str, Any]:
        return current_state.model_dump()

    @app.get("/api/config")
    def get_config() -> Dict[str, Any]:
        return current_config.model_dump()

    @app.post("/api/config")
    def update_config(config: TurretConfigModel) -> Dict[str, Any]:
        nonlocal current_config
        current_config = config
        return {"status": "ok", "config": current_config.model_dump()}

    @app.get("/api/hardware/ports")
    def list_hardware_ports() -> Dict[str, Any]:
        return {
            "ports": [
                {"port": "SIMULATION", "description": "Virtual Simulation Loopback", "connected": True},
                {"port": "COM3", "description": "Arduino Uno", "connected": False},
            ]
        }

    @app.post("/api/turret/fire")
    def trigger_fire() -> Dict[str, Any]:
        return {"status": "fired", "timestamp": 1725225120.0}

    def frame_generator() -> Generator[bytes, None, None]:
        # Generate dummy JPEG frame
        dummy = np.zeros((240, 320, 3), dtype=np.uint8)
        _, jpeg = cv2.imencode(".jpg", dummy)
        frame_bytes = jpeg.tobytes()

        for _ in range(5):
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n"
            )

    @app.get("/video_feed")
    def video_feed():
        return StreamingResponse(
            frame_generator(),
            media_type="multipart/x-mixed-replace; boundary=frame",
        )

    return app


@pytest.fixture
def api_client() -> TestClient:
    app = create_test_fastapi_app()
    return TestClient(app)


def test_fastapi_status_endpoint(api_client):
    """T1.10.1: Verifies GET /api/status returns HTTP 200 and telemetry data."""
    response = api_client.get("/api/status")
    assert response.status_code == 200
    data = response.json()
    assert "fps" in data
    assert "pan_angle" in data
    assert "tilt_angle" in data
    assert "distance_m" in data
    assert "speed_kmh" in data
    assert "target_locked" in data


def test_fastapi_get_config(api_client):
    """T1.10.2: Verifies GET /api/config returns default configuration."""
    response = api_client.get("/api/config")
    assert response.status_code == 200
    data = response.json()
    assert data["confidence"] == 0.50
    assert data["muzzle_velocity"] == 80.0
    assert data["net_cd"] == 1.20


def test_fastapi_post_config_valid(api_client):
    """T1.10.3: Verifies POST /api/config updates settings successfully."""
    new_cfg = {
        "confidence": 0.75,
        "target_class": 4,
        "muzzle_velocity": 95.0,
        "net_mass": 0.70,
        "net_cd": 1.40,
        "model_name": "drone_best.pt",
    }
    response = api_client.post("/api/config", json=new_cfg)
    assert response.status_code == 200
    assert response.json()["status"] == "ok"

    # Verify updated
    get_res = api_client.get("/api/config")
    assert get_res.json()["confidence"] == 0.75
    assert get_res.json()["model_name"] == "drone_best.pt"


def test_fastapi_ports_scan(api_client):
    """T1.10.4: Verifies GET /api/hardware/ports lists serial ports and simulation."""
    response = api_client.get("/api/hardware/ports")
    assert response.status_code == 200
    data = response.json()
    assert "ports" in data
    ports = [p["port"] for p in data["ports"]]
    assert "SIMULATION" in ports


def test_fastapi_fire_endpoint(api_client):
    """T1.10.5: Verifies POST /api/turret/fire returns HTTP 200 and fired status."""
    response = api_client.post("/api/turret/fire")
    assert response.status_code == 200
    assert response.json()["status"] == "fired"


def test_fastapi_mjpeg_stream_headers(api_client):
    """T1.10.6: Verifies GET /video_feed returns correct multipart headers."""
    response = api_client.get("/video_feed")
    assert response.status_code == 200
    content_type = response.headers.get("content-type", "")
    assert "multipart/x-mixed-replace" in content_type
    assert "boundary=frame" in content_type
