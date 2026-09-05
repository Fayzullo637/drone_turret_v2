## 2026-09-02T02:12:02Z

You are the Kalman Predictive Tracking Worker for drone_turret_v2 (Milestone M2).

Authoritative requirements file: C:\Users\User\teamwork_projects\drone_turret_v2\.agents\ORIGINAL_REQUEST.md
Scope document: C:\Users\User\teamwork_projects\drone_turret_v2\PROJECT.md
Spec reference: C:\Users\User\teamwork_projects\drone_turret_v2\.agents\survey_spec_explorer\handoff.md
Your metadata directory: C:\Users\User\teamwork_projects\drone_turret_v2\.agents\worker_m2_kalman

MANDATORY INTEGRITY WARNING:
DO NOT CHEAT. All implementations must be genuine. DO NOT hardcode test results, create dummy/facade implementations, or circumvent the intended task. A teamwork_preview_auditor will independently verify your work. Integrity violations WILL be detected and your work WILL be rejected.

EXCLUSIVE WRITE OWNERSHIP:
You own `drone_turret/tracking/` exclusively.

MISSION:
1. Implement `drone_turret/tracking/__init__.py`.
2. Implement `drone_turret/tracking/kalman_filter.py`:
   - Constant Acceleration (CA) 6-state Kalman filter using `cv2.KalmanFilter` (state vector: `[x, y, dx, dy, ddx, ddy]`, measurement vector: `[z_x, z_y]`).
   - Dynamic dt handling: recalculate state transition matrix F(dt) and process noise covariance Q(dt) per frame based on actual measured elapsed time between updates.
   - Robust measurement update with covariance update.
   - Forward trajectory projection: method `predict_trajectory(dt_horizon=1.0, num_steps=20)` computing predicted pixel points `[(x_0, y_0), (x_1, y_1), ...]` for t in [0.1, 2.0]s.
   - Target loss / occlusion coasting: when target is not detected, perform predict-only step, increment `coast_frames`, maintain state estimate for >= 10-15 frames before transitioning to LOST.
   - Speed calculation: convert estimated velocity `(dx, dy)` in pixels/sec and distance `d` in meters into real world velocity in m/s and km/h: `v_kmh = (sqrt(vx^2 + vy^2 + vz^2)) * 3.6`.
   - Comprehensive state dataclasses and reset mechanisms.
3. Verify your implementation with unit/physics tests, and write your report to `C:\Users\User\teamwork_projects\drone_turret_v2\.agents\worker_m2_kalman\handoff.md`.

Send message back when complete.
