# Project: drone_turret_v2 (AI Interceptor Turret with Ballistic Net Launcher)

## Architecture
The system is designed as a modular, high-throughput, cyber-physical AI guidance architecture for anti-drone turret interception:
```
[Video Feed / Camera] ──> [YOLOv8 + ByteTrack] ──> [6-State Kalman Filter] ──> [Target Kinematics]
                                                                                      │
[LiDAR / BBox Optics] ────────────────────────────────────────────────────────────────┼──> [RK4 Ballistics + Newton-Raphson]
                                                                                      │                 │
                                                                                      ▼                 ▼
[FastAPI Backend + Web HUD] <── [Live Stream & Telemetry Hub] <── [Pipeline Coordinator] <── [Lead Aim Point (Pan/Tilt)]
                                                                                      │                 │
                                                                                      ▼                 ▼
                                                            [Simulation Mode / Arduino Serial] <── [PID Controller]
```

## Feature Inventory
| # | Feature | Description | Milestone | Source |
|---|---------|-------------|-----------|--------|
| F1 | Real-time Drone Detection | YOLOv8 inference on camera stream, class filtering, confidence threshold | M1 | ORIGINAL_REQUEST §R1 |
| F2 | ByteTrack Persistent Tracking | Multi-object association, track continuity, track ID display, single-target locking | M1 | ORIGINAL_REQUEST §R1 |
| F3 | Model Hot-Swapping | Dynamic runtime switching between `yolov8n.pt` and `drone_best.pt` | M1 | ORIGINAL_REQUEST §R1 |
| F4 | 6-State Kalman Filter | cv2.KalmanFilter state vector [x, y, dx, dy, ddx, ddy], dynamic dt transition matrix F(dt) | M2 | ORIGINAL_REQUEST §R2 |
| F5 | Multi-Horizon Trajectory Projection | Forward trajectory prediction for t in [0.1, 2.0]s, dashed line rendering | M2 | ORIGINAL_REQUEST §R2 |
| F6 | Target Loss / Occlusion Coasting | Predict-only mode during detection loss for >= 10 frames without track loss | M2 | ORIGINAL_REQUEST §R2 |
| F7 | Speed Calculation in km/h | Target velocity estimation in 3D/pixels converted to km/h via range/known size | M2 | ORIGINAL_REQUEST §R2 |
| F8 | RK4 Numerical Ballistic Integrator | 4th-order Runge-Kutta numerical integration for net projectile with gravity & drag | M3 | ORIGINAL_REQUEST §R3 |
| F9 | Dynamic Drag & Net Expansion Model | Aerodynamic drag d(v_p)/dt = -(1/2m)*rho*Cd*A(t)*|v_p|*v_p + g with Cd=1.1-1.5 | M3 | ORIGINAL_REQUEST §R3 |
| F10 | Newton-Raphson Intercept Solver | Root-finding algorithm computing lead time t_int and drop-compensated pan/tilt lead angles | M3 | ORIGINAL_REQUEST §R3 |
| F11 | Intercept Lead Point Visualization | HUD lead crosshair marker distinct from drone centroid with time-to-intercept readout | M3 | ORIGINAL_REQUEST §R3 |
| F12 | LiDAR Serial Parser | 9-byte binary packet parser for TFMini/TF-Luna with checksum validation | M4 | ORIGINAL_REQUEST §R4 |
| F13 | Passive Optical Bounding Box Ranging | Distance = (known_drone_size * focal_length) / bbox_width_pixels with drone presets | M4 | ORIGINAL_REQUEST §R4 |
| F14 | Dual-Axis Pan/Tilt PID Controller | Discrete PID with anti-windup clamping, deadband filtering, filtered derivative, 0-180 absolute angles | M4 | ORIGINAL_REQUEST §R5 |
| F15 | Hardware & Virtual Simulation Mode | PySerial Arduino communication with seamless software simulation fallback | M4 | ORIGINAL_REQUEST §R5 |
| F16 | FastAPI Backend & REST API | REST endpoints for config, telemetry, camera selection, serial ports, and firing | M5 | ORIGINAL_REQUEST §R6 |
| F17 | Low-Latency MJPEG Streaming Feed | `/video_feed` multipart stream with tactical HUD overlay (reticle, lead point, trajectory, badges) | M5 | ORIGINAL_REQUEST §R6 |
| F18 | Responsive Cyber Tactical Web GUI | Single-page modern HUD frontend with dynamic parameter sliders, live stream, and status widgets | M5 | ORIGINAL_REQUEST §R6 |
| F19 | Modular Architecture (>= 5 modules) | Clean modular package structure in `drone_turret/` with separation of concerns | M6 | ORIGINAL_REQUEST §R7 |
| F20 | Arduino .ino Firmware | Arduino Uno firmware with non-blocking serial parser, servo drive, failsafe watchdog, and fire trigger | M6 | ORIGINAL_REQUEST §R7 |
| F21 | Requirements & Quickstart Documentation | `requirements.txt` and comprehensive `README.md` with installation, CLI usage, and API docs | M6 | ORIGINAL_REQUEST §R7 |
| F22 | 100% E2E Automated Verification | Opaque-box 5-tier test suite verifying all acceptance criteria across synthetic fixtures | M-FINAL | ORIGINAL_REQUEST §Acceptance Criteria |

## Milestones
| # | Name | Scope | Dependencies | Status |
|---|------|-------|-------------|--------|
| M-TEST | E2E Testing Track | Complete 4-Tier test suite, synthetic video/trajectory fixtures, mock serial devices, physics benchmarks | none | IN_PROGRESS |
| M1 | Vision & Tracking Engine | Model files setup, YOLOv8 detector wrapper, ByteTrack single-target lock, model hot-swapping | none | IN_PROGRESS |
| M2 | 6-State Kalman Predictive Tracker | 6-state CA Kalman filter, dynamic dt, trajectory projection, 10+ frame coasting, speed km/h | M1 (interface) | PLANNED |
| M3 | RK4 Ballistics & Intercept Solver | RK4 numerical integrator, expanding net drag model, Newton-Raphson intercept solver, lead point math | none | PLANNED |
| M4 | Distance Sensors & PID Control | TFMini LiDAR binary parser, passive bbox distance estimator, dual-axis PID controller, Arduino/Mock serial | none | PLANNED |
| M5 | FastAPI Web & Tactical HUD | FastAPI REST API, MJPEG stream generator with HUD overlays, modern responsive HTML/JS/CSS Web GUI | M1, M2, M3, M4 | PLANNED |
| M6 | Integration, Firmware, Docs | Pipeline coordinator, CLI runner, Arduino .ino firmware, requirements.txt, comprehensive README.md | M1..M5 | PLANNED |
| M-FINAL | E2E Validation & Adversarial Hardening | Pass 100% of E2E test suite (Tiers 1-4) + Tier 5 white-box adversarial stress & gap closure | M-TEST, M6 | PLANNED |

## Interface Contracts

### Vision ↔ Tracking (`vision` ↔ `tracking`)
```python
@dataclass
class Detection:
    box: Tuple[int, int, int, int]  # x1, y1, x2, y2 in pixels
    confidence: float
    class_id: int
    class_name: str
    track_id: Optional[int] = None

class BaseTracker(ABC):
    @abstractmethod
    def update(self, frame: np.ndarray, detections: List[Detection], dt: float) -> Optional[TrackedTarget]: ...
```

### Tracking ↔ Ballistics (`tracking` ↔ `ballistics`)
```python
@dataclass
class TargetState:
    pos_2d: Tuple[float, float]        # x, y in pixels
    vel_2d: Tuple[float, float]        # vx, vy in px/s
    acc_2d: Tuple[float, float]        # ax, ay in px/s^2
    pos_3d: Tuple[float, float, float] # X, Y, Z in meters (turret-centric)
    vel_3d: Tuple[float, float, float] # vX, vY, vZ in m/s
    speed_kmh: float
    is_coasting: bool
    coast_frames: int

@dataclass
class InterceptSolution:
    reachable: bool
    t_intercept: float                 # time to intercept in seconds
    lead_pos_3d: Tuple[float, float, float] # X, Y, Z in meters
    aim_pan_deg: float                 # absolute pan angle [0, 180]
    aim_tilt_deg: float                # absolute tilt angle [0, 180]
    lead_pixel_xy: Tuple[int, int]     # (x, y) lead crosshair in screen pixels
    drop_m: float                      # vertical gravity + drag drop in meters
```

### Distance Sensor & Kinematics (`sensors` ↔ `tracking`/`ballistics`)
```python
class DistanceEstimator:
    def get_distance(self, bbox: Tuple[int, int, int, int], frame_shape: Tuple[int, int], lidar_dist: Optional[float] = None) -> Tuple[float, str]:
        """Returns (distance_meters, source_name: 'lidar' | 'optical')."""
```

### PID Controller ↔ Hardware / Serial (`control` ↔ `comms`)
```python
class TurretController:
    def update_lead_target(self, target_pan_deg: float, target_tilt_deg: float, dt: float) -> Tuple[float, float]:
        """Calculates discrete PID step and returns (cmd_pan_deg, cmd_tilt_deg) clamped to [0, 180]."""
```

## Code Layout
```
drone_turret_v2/
├── drone_turret/                      # Main Python package (>= 6 modules)
│   ├── __init__.py
│   ├── vision/
│   │   ├── __init__.py
│   │   ├── camera.py                  # Multi-threaded camera capture & device scanning
│   │   └── detector.py                # YOLOv8 + ByteTrack inference & model swapping
│   ├── tracking/
│   │   ├── __init__.py
│   │   └── kalman_filter.py           # 6-state CA cv2.KalmanFilter with dt handling & coasting
│   ├── ballistics/
│   │   ├── __init__.py
│   │   └── calculator.py              # RK4 variable-drag integrator & Newton-Raphson solver
│   ├── sensors/
│   │   ├── __init__.py
│   │   ├── lidar.py                   # TFMini/TF-Luna 9-byte serial parser & checksum
│   │   └── distance.py                # Optical pinhole bbox distance estimation
│   ├── control/
│   │   ├── __init__.py
│   │   └── pid.py                     # Dual-axis discrete PID with anti-windup & deadband
│   ├── comms/
│   │   ├── __init__.py
│   │   └── serial_comm.py             # PySerial Arduino communicator & mock transport
│   ├── web/
│   │   ├── __init__.py
│   │   ├── app.py                     # FastAPI application, REST endpoints, MJPEG stream
│   │   └── stream.py                  # Video streamer with tactical HUD renderer
│   ├── coordinator.py                 # Central pipeline coordinator wiring all subsystems
│   └── config.py                      # Pydantic system configuration & presets
├── firmware/
│   └── drone_turret_firmware.ino      # Arduino Uno non-blocking firmware
├── static/
│   ├── index.html                     # Tactical Cyber HUD UI
│   ├── app.js                         # Web client JS (telemetry, sliders, controls)
│   └── styles.css                     # Dark military HUD styling
├── models/                            # YOLO model directory (yolov8n.pt, drone_best.pt)
├── tests/                             # Comprehensive 5-Tier test suite
│   ├── conftest.py                    # Shared test fixtures, mocks, generators
│   ├── fixtures/
│   │   ├── synthetic_video.py         # Synthetic frame & kinematic target generator
│   │   ├── mock_serial.py             # Virtual serial port & fault injector
│   │   └── physics_benchmarks.py      # Analytical closed-form physics benchmarks
│   ├── tier1_features/                # Unit feature tests (>= 5 per feature)
│   ├── tier2_boundaries/              # Boundary and corner case tests (>= 5 per feature)
│   ├── tier3_pairwise/                # Cross-feature integration tests
│   ├── tier4_scenarios/               # Real-world field combat scenarios
│   └── tier5_stress/                  # Adversarial stress and soak tests
├── main.py                            # CLI entry point to launch web server
├── requirements.txt                   # Production Python dependencies
└── README.md                          # Comprehensive documentation & setup instructions
```
