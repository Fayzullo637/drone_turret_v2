import cv2
import numpy as np
from fastapi import FastAPI, Response
from fastapi.responses import StreamingResponse, JSONResponse
from fastapi.testclient import TestClient
from pydantic import BaseModel, Field
import time

app = FastAPI(title="Drone Turret AI API")

# Shared application state
class AppConfig(BaseModel):
    confidence: float = Field(default=0.6, ge=0.0, le=1.0)
    model_file: str = "drone_best.pt"
    target_class: int = 0
    muzzle_velocity: float = Field(default=80.0, gt=0.0)
    projectile_mass: float = Field(default=0.8, gt=0.0)
    drag_coeff: float = Field(default=0.4, ge=0.0)

class SystemState:
    def __init__(self):
        self.config = AppConfig()
        self.fps = 30.0
        self.locked_track_id = None
        self.pan_angle = 90.0
        self.tilt_angle = 90.0
        self.target_distance = 25.5
        self.target_speed_kmh = 120.0
        self.lead_point = {"x": 320, "y": 240}
        self.arduino_connected = False
        self.lidar_connected = False

state = SystemState()

@app.get("/api/status")
def get_status():
    return {
        "fps": state.fps,
        "locked_id": state.locked_track_id,
        "pan": state.pan_angle,
        "tilt": state.tilt_angle,
        "distance_m": state.target_distance,
        "speed_kmh": state.target_speed_kmh,
        "lead_point": state.lead_point,
        "arduino_connected": state.arduino_connected,
        "lidar_connected": state.lidar_connected
    }

@app.get("/api/config")
def get_config():
    return state.config.model_dump()

@app.post("/api/config")
def set_config(new_config: AppConfig):
    state.config = new_config
    return {"status": "ok", "config": state.config.model_dump()}

def frame_generator():
    for _ in range(5):
        frame = np.zeros((240, 320, 3), dtype=np.uint8)
        cv2.putText(frame, "TEST FEED", (50, 120), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        _, jpeg = cv2.imencode(".jpg", frame)
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + jpeg.tobytes() + b"\r\n")

@app.get("/video_feed")
def video_feed():
    return StreamingResponse(
        frame_generator(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

def test_fastapi_endpoints():
    print("--- FASTAPI TESTCLIENT & E2E VERIFICATION ---")
    client = TestClient(app)
    
    # 1. Test /api/status
    res = client.get("/api/status")
    assert res.status_code == 200
    data = res.json()
    print(f"Status response: {data}")
    assert "fps" in data
    assert data["pan"] == 90.0
    
    # 2. Test /api/config GET and POST
    res_cfg = client.get("/api/config")
    assert res_cfg.status_code == 200
    assert res_cfg.json()["confidence"] == 0.6
    
    # Update config
    update_res = client.post("/api/config", json={
        "confidence": 0.75,
        "model_file": "yolov8n.pt",
        "target_class": 3,
        "muzzle_velocity": 85.0,
        "projectile_mass": 0.6,
        "drag_coeff": 1.2
    })
    assert update_res.status_code == 200
    assert update_res.json()["config"]["confidence"] == 0.75
    print(f"Config updated successfully: {update_res.json()}")

    # 3. Test validation error (negative muzzle velocity)
    bad_res = client.post("/api/config", json={"muzzle_velocity": -10.0})
    assert bad_res.status_code == 422 # Validation error
    print("Validation error correctly returned 422 on negative muzzle velocity!")

    # 4. Test /video_feed stream
    stream_res = client.get("/video_feed")
    assert stream_res.status_code == 200
    assert "multipart/x-mixed-replace" in stream_res.headers["content-type"]
    content = stream_res.content
    assert b"--frame" in content
    assert b"image/jpeg" in content
    print(f"Video feed stream tested successfully (received {len(content)} bytes)!")

if __name__ == '__main__':
    test_fastapi_endpoints()
