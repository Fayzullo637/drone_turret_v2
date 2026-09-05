## 2026-09-01T21:12:02Z

You are the Vision & Tracking Worker for drone_turret_v2 (Milestone M1).

Authoritative requirements file: C:\Users\User\teamwork_projects\drone_turret_v2\.agents\ORIGINAL_REQUEST.md
Scope document: C:\Users\User\teamwork_projects\drone_turret_v2\PROJECT.md
Spec reference: C:\Users\User\teamwork_projects\drone_turret_v2\.agents\survey_spec_explorer\handoff.md
v1 Prototype directory: C:\Users\User\teamwork_projects\drone_ai_detector\
Your metadata directory: C:\Users\User\teamwork_projects\drone_turret_v2\.agents\worker_m1_vision

MANDATORY INTEGRITY WARNING:
DO NOT CHEAT. All implementations must be genuine. DO NOT hardcode test results, create dummy/facade implementations, or circumvent the intended task. A teamwork_preview_auditor will independently verify your work. Integrity violations WILL be detected and your work WILL be rejected.

EXCLUSIVE WRITE OWNERSHIP:
You own `models/` and `drone_turret/vision/` exclusively.

MISSION:
1. Create `models/` directory in `drone_turret_v2/` and copy `yolov8n.pt` and `drone_best.pt` (and any other relevant weights like `yolov8_drone.pt`) from `C:\Users\User\teamwork_projects\drone_ai_detector\models\` (or root of v1) into `C:\Users\User\teamwork_projects\drone_turret_v2\models\`.
2. Implement `drone_turret/vision/__init__.py`.
3. Implement `drone_turret/vision/camera.py`:
   - Multi-threaded OpenCV VideoCapture with fallback to synthetic/test frame source.
   - Camera device auto-discovery (DirectShow on Windows via `cv2.CAP_DSHOW` checking indices 0..4 non-blocking).
   - Thread-safe frame reading, latest frame caching, FPS tracking, and graceful release.
4. Implement `drone_turret/vision/detector.py`:
   - YOLOv8 inference wrapper with ByteTrack tracking integration (`model.track(frame, persist=True, tracker="bytetrack.yaml")`).
   - Dynamic model hot-swapping between `yolov8n.pt` and `drone_best.pt` without restarting the application.
   - Target class filtering (e.g. classes 0..4 for drone_best: shahed, reaper, mavic, etc. or COCO classes), confidence thresholding, and single-target lock retention (`locked_track_id` priority or largest bbox area).
   - Output clean `Detection` dataclasses with box `(x1, y1, x2, y2)`, confidence, class_id, class_name, track_id, center `(cx, cy)`, and width/height.
5. Verify your code with a quick standalone test or unit test, and write your report to `C:\Users\User\teamwork_projects\drone_turret_v2\.agents\worker_m1_vision\handoff.md`.
