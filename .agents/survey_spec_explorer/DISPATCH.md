## 2026-09-01T21:07:31Z

You are the Spec & Architecture Explorer for drone_turret_v2.

Your metadata directory is: C:\Users\User\teamwork_projects\drone_turret_v2\.agents\survey_spec_explorer
Authoritative requirements file: C:\Users\User\teamwork_projects\drone_turret_v2\.agents\ORIGINAL_REQUEST.md

Investigate and document the complete specification and mathematical/architectural requirements:
1. Mathematical foundations:
   - Kalman Filter: cv2.KalmanFilter 6-state [x, y, dx, dy, ddx, ddy], measurement matrix H (2x6 or 4x6), state transition matrix F(dt), process noise Q, measurement noise R, covariance P, dt handling, prediction for t in [0.1, 2.0]s, track coasting/loss handling (>=10 frames).
   - Ballistic Calculator: RK4 numerical integration for projectile trajectory with drag d(v_p)/dt = -(1/2m)*rho*Cd*A(t)*|v_p|*v_p + g, variable Cd (1.1 - 1.5 as net expands), net mass m, air density rho=1.225 kg/m^3, area A(t). Newton-Raphson root finding to find intercept time t_intercept and lead aim angles (pan, tilt) where net position p_net(t) matches predicted drone position p_drone(t).
   - Distance estimation: LiDAR serial reader (TFMini / TF-Luna packet parsing) and passive bbox estimation formula: distance = (known_drone_size * focal_length) / bbox_size_pixels. Speed in km/h calculation.
   - PID controller for pan/tilt servos: discrete PID with clamping, deadband, anti-windup, simulation mode outputting absolute angles [0, 180] deg.
   - FastAPI backend: REST API + WebSocket/MJPEG stream endpoints, configuration management, hardware toggles, target lock/tracking status.
   - Arduino .ino firmware specification: Serial command protocol, servo control.

Write your findings to C:\Users\User\teamwork_projects\drone_turret_v2\.agents\survey_spec_explorer\handoff.md.
When finished, send a message back to parent orchestrator.
