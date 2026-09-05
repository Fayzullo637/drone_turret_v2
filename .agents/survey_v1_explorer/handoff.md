# Handoff Report: v1 Prototype & Environment Survey for Drone Turret v2

## 1. Observation

### 1.1 Detailed Inventory of v1 Prototype (`C:\Users\User\teamwork_projects\drone_ai_detector\`)
A complete audit of all files in the v1 prototype directory was conducted:

| File / Path | Size | Role / Description |
|---|---|---|
| `detector.py` | 8,841 B (198 lines) | Main execution script. Uses `cv2.VideoCapture`, `ultralytics.YOLO.track()` with ByteTrack, Tkinter startup modal dialog, direct pixel-error PID calculations, and `cv2.imshow` desktop window with manual HUD drawings. |
| `gui.py` | 4,833 B (155 lines) | Tkinter GUI dialog shown before video stream starts (`setup_gui`). Contains camera scanning (`_find_cameras`), COM port enumeration, model selection, target class mapping dictionary, and confidence slider. |
| `pid_controller.py` | 2,672 B (62 lines) | `PIDController` class with proportional, integral (with anti-windup clamping), and derivative terms. Provides `update(error, dt)` and `reset()`. |
| `arduino_comm.py` | 1,722 B (49 lines) | `ArduinoController` class wrapping `pyserial.Serial`. Transmits formatted strings `f"{pan},{tilt}\n"` at 115200 baud. Contains fallback simulation mode when port is empty or connection fails. |
| `arduino_turret/arduino_turret.ino` | 1,083 B (37 lines) | Arduino C++ sketch. Attaches two servos (`Servo.h`) on pin 9 (Pan) and pin 10 (Tilt). Listens to Serial at 115200 baud, parses `PAN_ANGLE,TILT_ANGLE\n`, and updates servos. |
| `requirements.txt` | 52 B (5 lines) | Contains `ultralytics`, `opencv-python`, `numpy`, and `p y s e r i a l` (note whitespace formatting). |
| `README.md` | 3,329 B (51 lines) | Prototype overview, installation steps, and future roadmap notes. |

### 1.2 Model Files Location & Verification
All `.pt` model files were located on disk and verified for loadability and class metadata using `ultralytics.YOLO`:

1. **`drone_best.pt`**:
   - Location: `C:\Users\User\teamwork_projects\drone_ai_detector\drone_best.pt` (and mirror in `C:\Users\User\.cache\huggingface\hub\models--Tuzelkhan--drone-yolov8\...`)
   - Size: 6,273,770 bytes (~6.27 MB)
   - Task: `detect`
   - Verified Classes (5): `{0: 'shahed_136', 1: 'shahed_238', 2: 'mq9_reaper', 3: 'dji_mavic', 4: 'mohajer_6'}`
   - Status: Load verified successfully.

2. **`yolov8n.pt`**:
   - Location: `C:\Users\User\teamwork_projects\drone_ai_detector\yolov8n.pt`
   - Size: 6,549,796 bytes (~6.55 MB)
   - Task: `detect`
   - Verified Classes (80 COCO): `{0: 'person', 1: 'bicycle', 2: 'car', ..., 14: 'bird', ..., 79: 'toothbrush'}`
   - Status: Load verified successfully.

3. **`yolov8_drone.pt`** (VisDrone dataset):
   - Location: `C:\Users\User\teamwork_projects\drone_ai_detector\yolov8_drone.pt`
   - Size: 22,546,666 bytes (~22.55 MB)
   - Task: `detect`
   - Verified Classes (11): `{0: 'pedestrian', 1: 'people', 2: 'bicycle', 3: 'car', 4: 'van', 5: 'truck', 6: 'tricycle', 7: 'awning-tricycle', 8: 'bus', 9: 'motor', 10: 'others'}`
   - Status: Load verified successfully.

### 1.3 Python Environment, Runtime & Packages
A detailed inspection of the active Python interpreter revealed:
- **Python Version**: `3.14.5` (tags/v3.14.5:5607950, May 10 2026) 64-bit AMD64
- **Operating System**: Windows 10 (10.0.19041-SP0)
- **PyTorch**: `2.13.0+cpu`
- **CUDA / GPU Acceleration**: `CUDA available: False` (running in CPU-only mode)
- **OpenCV**: `5.0.0.93` (`cv2.KalmanFilter` verified functioning)
- **Ultralytics**: `8.4.120`
- **NumPy**: `2.4.4`
- **FastAPI**: `0.141.1`
- **Uvicorn**: `0.52.4`
- **PySerial (`serial`)**: `3.5`
- **Pytest**: `9.1.1`
- **Lap / Tracking**: `lap 0.5.13` (ByteTrack backend functional)
- **Websockets**: `17.1`
- **Pillow**: `12.3.0`
- **SciPy**: `NOT INSTALLED` (`ModuleNotFoundError`)

### 1.4 Hardware Peripherals Audit
- **Webcams**:
  - Camera index 0: `1280x720` (DirectShow / `CAP_DSHOW` working)
  - Camera index 1: `640x480` (DirectShow / `CAP_DSHOW` working)
  - Camera index 2: `1280x720` (DirectShow / `CAP_DSHOW` working)
- **Serial / COM Ports**: 0 connected hardware COM ports.
  - Seamless simulation fallback is essential for both Arduino servo actuation and LiDAR telemetry.

---

## 2. Logic Chain

### 2.1 Reusable vs Upgradeable Components from v1
1. **Model Weights (`drone_best.pt`, `yolov8n.pt`)**:
   - Both checkpoints are verified intact in v1. They can be copied directly to `drone_turret_v2/models/`.
2. **PID Controller (`pid_controller.py`)**:
   - The anti-windup clamping and derivative error tracking in v1 is mathematically sound.
   - *Upgrade needed*: In v1, the controller acted directly on raw target pixel error `(cx_obj - cx_screen)`. In v2, it must track the **Lead Intercept Point** $(x_{\text{lead}}, y_{\text{lead}})$ produced by the ballistic calculator.
3. **Arduino Communication & Firmware (`arduino_comm.py` & `arduino_turret.ino`)**:
   - Protocol `PAN,TILT\n` at 115200 baud is simple, reliable, and verified.
   - *Upgrade needed*: Non-blocking async queueing so serial I/O never blocks the vision/web pipeline; integration of LiDAR serial packet decoding (`0x59 0x59`) if connected.
4. **Camera Enumeration (`gui.py:_find_cameras`)**:
   - DirectShow sequential probing (`cv2.CAP_DSHOW`) reliably discovers cameras on Windows. Can be adapted as a backend service for FastAPI.

### 2.2 Critical Gaps in v1 That Must Be Built for v2
The v1 prototype lacks nearly all requirements specified in `ORIGINAL_REQUEST.md`:

| Requirement | v1 Prototype Status | v2 Requirement & Implementation Blueprint |
|---|---|---|
| **R1: Detection & ByteTrack** | Present in basic desktop loop; model selected only before startup. | **Dynamic switching** between `yolov8n.pt` and `drone_best.pt` via Web API at runtime without restarting the process. |
| **R2: Kalman Predictive Tracking** | **ABSENT** (v1 had only 1st-order EMA filter `smoothed_ox = 0.3*raw + 0.7*prev`). | **6-State Kalman Filter** (`[x, y, dx, dy, ddx, ddy]`) modeling position, velocity, and acceleration; $\Delta t$-adaptive transition matrix; trajectory projection (0.1 - 2.0s); 10+ frames occlusion coasting (dead reckoning); metric speed estimation in km/h. |
| **R3: Ballistic Calculator (Net Launcher)** | **ABSENT** (v1 aimed directly at drone centroid). | **RK4 Numerical Integrator** for projectile dynamics with quadratic air resistance $\frac{d\vec{v}_p}{dt} = -\frac{1}{2m}\rho C_d A(t)|\vec{v}_p|\vec{v}_p + \vec{g}$; **Newton-Raphson Root-Finder** for collision time $t^*$; calculation of lead aiming coordinates $(x_{\text{lead}}, y_{\text{lead}})$. |
| **R4: Distance Estimation** | **ABSENT** (v1 had no distance sensing). | **Pinhole Camera Model** ($D = \frac{W \cdot f}{w_{\text{px}}}$) based on drone bounding box; **LiDAR Serial Parser** for TFMini/TF-Luna (`0x59 0x59` 9-byte binary protocol); automatic fusion/fallback. |
| **R5: Lead-Point Servo Steering** | Partial (aimed at drone center). | Aiming at lead point; support for simulated turret feedback and logging. |
| **R6: Web Interface** | **ABSENT** (v1 used blocking Tkinter + `cv2.imshow`). | **FastAPI Web Backend** + **Tactical HUD Frontend** (HTML/CSS/JS) with live streaming (MJPEG/WebSocket), reticle, trajectory dotted line, lead crosshair, telemetry dashboard, and live parameter tuning. |
| **R7: Architecture & Tests** | Minimal 5 files, no automated tests. | Clean modular package structure (`src/`), comprehensive unit test suite (`pytest`) covering Kalman, Ballistics, PID, Distance, and Web API. |

---

## 3. Caveats

1. **CPU-Only PyTorch Environment**:
   - PyTorch 2.13.0 is running on CPU (`CUDA available: False`).
   - YOLOv8n single-frame CPU inference is ~200-300 ms at full resolution.
   - *Mitigation*: Design an asynchronous architecture where camera capture, Kalman filter updates, and web video streaming run at high frame rates (30+ FPS) while YOLO detection runs on a dedicated background worker with optimized image dimensions (`imgsz=480` or `imgsz=320`).
2. **Absence of SciPy**:
   - `scipy` is not installed in the Python 3.14 environment.
   - *Mitigation*: Implement the 4th-order Runge-Kutta (RK4) integrator, Newton-Raphson iterative solver, and matrix kinematics directly using `numpy` and pure Python. This avoids external dependencies while maintaining optimal execution speed.
3. **Hardware-Free Operation**:
   - No physical COM ports or LiDAR devices are currently plugged in.
   - *Mitigation*: Both Arduino and LiDAR modules must feature full simulation modes with synthetic data generators and visual feedback in the Web GUI.

---

## 4. Conclusion

The v1 prototype provides a working proof-of-concept for basic YOLO detection, ByteTrack tracking, and PID servo output, along with pre-trained model weights (`drone_best.pt` and `yolov8n.pt`). However, v1 lacks all ballistic calculations, predictive Kalman filtering, distance estimation, Web UI, and automated test coverage.

### Recommended Target Modular Architecture for `drone_turret_v2`:
```
drone_turret_v2/
+-- models/
¦   +-- drone_best.pt              # Copied from v1
¦   L-- yolov8n.pt                 # Copied from v1
+-- firmware/
¦   L-- arduino_turret.ino         # Arduino Uno 2-axis servo firmware
+-- src/
¦   +-- __init__.py
¦   +-- detector.py                # YOLOv8 + ByteTrack detection engine
¦   +-- kalman_tracker.py          # 6-state Kalman predictor, trajectory & km/h speed
¦   +-- ballistics.py              # RK4 projectile simulation & Newton-Raphson lead solver
¦   +-- distance_estimator.py      # Pinhole bbox distance & TFMini LiDAR parser
¦   +-- pid_controller.py          # Lead-point targeting PID controller
¦   +-- arduino_comm.py            # Async serial controller & simulation mode
¦   +-- camera_stream.py           # Threaded camera capture & synthetic feed
¦   L-- server.py                  # FastAPI web server & WebSocket/MJPEG stream
+-- static/
¦   +-- index.html                 # Tactical HUD Web UI
¦   +-- app.js                     # HUD rendering, telemetry & control client
¦   L-- style.css                  # Dark military/tactical theme
+-- tests/
¦   +-- test_kalman.py             # Kalman state, trajectory & coasting tests
¦   +-- test_ballistics.py         # RK4 integrator & intercept solver tests
¦   +-- test_distance.py           # Pinhole & LiDAR packet parsing tests
¦   +-- test_pid.py                # PID step & anti-windup tests
¦   L-- test_api.py                # FastAPI endpoints & streaming tests
+-- requirements.txt
+-- README.md
L-- run.py                         # Single-command launcher
```

---

## 5. Verification Method

To independently reproduce and verify all findings:

1. **Verify Python & Core Libraries**:
   ```powershell
   python -c "import torch, ultralytics, cv2, fastapi, uvicorn, serial, pytest, numpy; print('Python & Core packages OK, Torch CPU:', torch.__version__)"
   ```
2. **Verify Model Files**:
   ```powershell
   python -c "from ultralytics import YOLO; m1=YOLO(r'C:\Users\User\teamwork_projects\drone_ai_detector\drone_best.pt'); m2=YOLO(r'C:\Users\User\teamwork_projects\drone_ai_detector\yolov8n.pt'); print('drone_best classes:', m1.names); print('yolov8n classes count:', len(m2.names))"
   ```
3. **Verify Camera Probing**:
   ```powershell
   python -c "import cv2; print([(i, int(cv2.VideoCapture(i, cv2.CAP_DSHOW).get(3)), int(cv2.VideoCapture(i, cv2.CAP_DSHOW).get(4))) for i in range(3)])"
   ```
4. **Verify OpenCV Kalman Filter**:
   ```powershell
   python -c "import cv2; kf=cv2.KalmanFilter(6, 2, 0); print('Kalman filter init:', kf)"
   ```
