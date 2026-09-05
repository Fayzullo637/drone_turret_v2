# E2E Test Infra: drone_turret_v2

## Test Philosophy
- Opaque-box, requirement-driven testing independent of implementation internals.
- Hardware-free deterministic execution utilizing high-fidelity synthetic video generators, mathematical oracles, and virtual serial transports.
- Methodology: Category-Partition + Boundary Value Analysis + Pairwise Combinatorial Testing + Real-World Tactical Scenarios.

## Feature Inventory & Test Coverage
| # | Feature | Source (Requirement) | Tier 1 (Unit) | Tier 2 (Boundary) | Tier 3 (Pairwise) | Tier 4 (Scenario) |
|---|---------|---------------------|:-------------:|:-----------------:|:-----------------:|:-----------------:|
| F1 | Real-time Drone Detection | ORIGINAL_REQUEST §R1 | ≥5 | ≥5 | ✓ | ✓ |
| F2 | ByteTrack Persistent Tracking | ORIGINAL_REQUEST §R1 | ≥5 | ≥5 | ✓ | ✓ |
| F3 | Model Hot-Swapping | ORIGINAL_REQUEST §R1 | ≥5 | ≥5 | ✓ | ✓ |
| F4 | 6-State Kalman Predictive Filter | ORIGINAL_REQUEST §R2 | ≥5 | ≥5 | ✓ | ✓ |
| F5 | Trajectory Prediction (0.1-2.0s) | ORIGINAL_REQUEST §R2 | ≥5 | ≥5 | ✓ | ✓ |
| F6 | Target Loss / Occlusion (>=10 frames)| ORIGINAL_REQUEST §R2 | ≥5 | ≥5 | ✓ | ✓ |
| F7 | Speed Calculation in km/h | ORIGINAL_REQUEST §R2 | ≥5 | ≥5 | ✓ | ✓ |
| F8 | RK4 Numerical Ballistic Integrator | ORIGINAL_REQUEST §R3 | ≥5 | ≥5 | ✓ | ✓ |
| F9 | Variable Aerodynamic Drag (Cd=1.1-1.5)| ORIGINAL_REQUEST §R3 | ≥5 | ≥5 | ✓ | ✓ |
| F10 | Newton-Raphson Intercept Solver | ORIGINAL_REQUEST §R3 | ≥5 | ≥5 | ✓ | ✓ |
| F11 | Intercept Lead Crosshair | ORIGINAL_REQUEST §R3 | ≥5 | ≥5 | ✓ | ✓ |
| F12 | LiDAR 9-byte Serial Decoder | ORIGINAL_REQUEST §R4 | ≥5 | ≥5 | ✓ | ✓ |
| F13 | Optical Bounding Box Ranging | ORIGINAL_REQUEST §R4 | ≥5 | ≥5 | ✓ | ✓ |
| F14 | Dual-Axis Pan/Tilt PID Controller | ORIGINAL_REQUEST §R5 | ≥5 | ≥5 | ✓ | ✓ |
| F15 | Simulation Mode & Serial Protocol | ORIGINAL_REQUEST §R5 | ≥5 | ≥5 | ✓ | ✓ |
| F16 | FastAPI Backend REST API | ORIGINAL_REQUEST §R6 | ≥5 | ≥5 | ✓ | ✓ |
| F17 | Low-Latency MJPEG Stream | ORIGINAL_REQUEST §R6 | ≥5 | ≥5 | ✓ | ✓ |
| F18 | Web HUD UI & Dynamic Tuning | ORIGINAL_REQUEST §R6 | ≥5 | ≥5 | ✓ | ✓ |

## Test Architecture & Directory Layout
```
tests/
├── conftest.py
├── fixtures/
│   ├── synthetic_video.py
│   ├── mock_serial.py
│   └── physics_benchmarks.py
├── tier1_features/
├── tier2_boundaries/
├── tier3_pairwise/
├── tier4_scenarios/
└── tier5_stress/
```

## Real-World Application Scenarios (Tier 4)
| # | Scenario | Features Exercised | Complexity |
|---|----------|--------------------|------------|
| 1 | High-speed 200 km/h Orthogonal Flyby | F1, F2, F4, F7, F8, F10, F14 | High |
| 2 | Direct Head-on Fast Dive (80m -> 10m) | F1, F4, F8, F10, F12, F13, F14 | Medium |
| 3 | High-G Evasive Sinusoidal Zigzag | F2, F4, F5, F10, F14 | High |
| 4 | 15-Frame Visual Occlusion & Recovery | F2, F4, F5, F6, F10 | High |
| 5 | Dual-Drone Trajectory Crossing & Lock Retention | F1, F2, F4, F14 | Medium |

## Coverage Thresholds
- Tier 1: ≥ 5 test cases per feature (> 50 test cases)
- Tier 2: ≥ 5 boundary test cases per feature (> 50 test cases)
- Tier 3: Pairwise cross-feature interactions (> 10 test cases)
- Tier 4: ≥ 5 realistic application scenarios
- Tier 5: Adversarial stress & memory soak testing
- **Total Minimum Target: ≥ 115+ automated test cases**
