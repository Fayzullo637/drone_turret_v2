# DISPATCH

## 2026-09-02T02:12:02Z
You are the Distance Sensors & PID Control Worker for drone_turret_v2 (Milestone M4).

Authoritative requirements file: C:\Users\User\teamwork_projects\drone_turret_v2\.agents\ORIGINAL_REQUEST.md
Scope document: C:\Users\User\teamwork_projects\drone_turret_v2\PROJECT.md
Spec reference: C:\Users\User\teamwork_projects\drone_turret_v2\.agents\survey_spec_explorer\handoff.md
v1 Prototype directory: C:\Users\User\teamwork_projects\drone_ai_detector\
Your metadata directory: C:\Users\User\teamwork_projects\drone_turret_v2\.agents\worker_m4_sensors_control

MANDATORY INTEGRITY WARNING:
DO NOT CHEAT. All implementations must be genuine. DO NOT hardcode test results, create dummy/facade implementations, or circumvent the intended task. A teamwork_preview_auditor will independently verify your work. Integrity violations WILL be detected and your work WILL be rejected.

EXCLUSIVE WRITE OWNERSHIP:
You own `drone_turret/sensors/`, `drone_turret/control/`, and `drone_turret/comms/` exclusively.

MISSION:
1. Implement `drone_turret/sensors/`:
   - `lidar.py`: TFMini / TF-Luna LiDAR 9-byte serial binary packet parser (`0x59 0x59 Dist_L Dist_H Strength_L Strength_H Temp_L Temp_H Checksum`), checksum verification, strength filtering (>= 100), and auto-resync after corrupted bytes.
   - `distance.py`: Optical distance estimation using pinhole model: `distance = (known_drone_size * focal_length) / bbox_width_pixels` with drone presets (DJI Mavic 3 = 0.35m, Shahed-136 = 2.5m, FPV = 0.22m, MQ-9 = 20m) and camera HFOV calculation. Fusion/fallback logic: use LiDAR when valid packet available, seamlessly fall back to optical bbox when LiDAR absent/disconnected.
2. Implement `drone_turret/control/`:
   - `pid.py`: Dual-axis discrete PID controller for Pan and Tilt servos. Aims at lead point $(\theta_{pan\_lead}, \theta_{tilt\_lead})$. Features: anti-windup clamping on integral term, deadband filtering ($\pm 0.3^\circ$), filtered derivative term, maximum slew rate limiting, output clamping to absolute angles $[0, 180]^\circ$.
3. Implement `drone_turret/comms/`:
   - `serial_comm.py`: Hardware serial communicator for Arduino Uno and LiDAR. Supports auto-detection of COM ports, non-blocking asynchronous transmission, formatting `P<pan>,T<tilt>\n` or `pan,tilt\n`, fire trigger `FIRE\n`, and software simulation mode when no hardware is connected.
4. Verify your implementation with unit tests, and write your report to `C:\Users\User\teamwork_projects\drone_turret_v2\.agents\worker_m4_sensors_control\handoff.md`.
