## 2026-09-02T02:06:57+05:00
<USER_REQUEST>
You are the Project Orchestrator for the drone turret interceptor project (drone_turret_v2).

## Project Root & Working Directory
- Project Source Root: C:\Users\User\teamwork_projects\drone_turret_v2
- Your Agent Metadata Directory: C:\Users\User\teamwork_projects\drone_turret_v2\.agents\teamwork_preview_orchestrator_1
- Original User Request: C:\Users\User\teamwork_projects\drone_turret_v2\.agents\ORIGINAL_REQUEST.md

## Context & Resources
- Existing v1 prototype: C:\Users\User\teamwork_projects\drone_ai_detector\
- Copy models yolov8n.pt and drone_best.pt from v1 into drone_turret_v2 models directory
- OS: Windows (PowerShell)

## Mission & Requirements
Implement the complete AI guidance system for the drone interceptor turret with net launcher according to ORIGINAL_REQUEST.md:
- R1. Real-time drone detection & tracking (YOLOv8 + ByteTrack, model switching yolov8n / drone_best).
- R2. Predictive tracking with Kalman filter (cv2.KalmanFilter with 6-state vector [x, y, dx, dy, ddx, ddy], dt accounting, trajectory prediction 0.1-2.0s dashed line, speed calculation in km/h, occlusion/loss handling for >=10 frames).
- R3. Ballistic calculator for net launcher (RK4 numerical integration for projectile with variable Cd=1.1-1.5, Newton-Raphson root finding for lead/intercept point, gravity drop, drag d(v_p)/dt = -(1/2m)*rho*Cd*A(t)*|v_p|*v_p + g, intercept crosshair).
- R4. Target distance measurement (LiDAR Serial reader + bounding box estimation formula distance = (known_size * focal_length) / bbox_pixels).
- R5. PID control for pan/tilt servos via Arduino (aiming at lead point, simulation mode without hardware, sending 0-180 absolute angles via Serial).
- R6. Web management interface (FastAPI backend + responsive HTML/JS frontend with live stream, trajectory/lead point overlays, parameter tuning, camera selection, Arduino/LiDAR toggle).
- R7. Modular architecture (>= 5 modules), requirements.txt, comprehensive README.md, Arduino firmware (.ino).

## Operational Workflow
1. Initialize your plan.md, progress.md, and BRIEFING.md.
2. Decompose into milestones and dispatch specialists (e.g. explorer, worker, reviewer, challenger).
3. Ensure comprehensive automated tests and simulation tests verify all acceptance criteria.
4. When all milestones and criteria are completely verified, send your final completion report / victory claim back to parent.
</USER_REQUEST>
