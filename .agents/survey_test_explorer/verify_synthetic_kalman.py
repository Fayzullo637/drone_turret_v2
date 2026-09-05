import cv2
import numpy as np
import math

class SyntheticVideoGenerator:
    """Generates synthetic OpenCV frames with ground-truth target motion."""
    def __init__(self, width=640, height=480, fps=30):
        self.width = width
        self.height = height
        self.fps = fps
        self.dt = 1.0 / fps
        self.frame_count = 0
        
    def generate_frame(self, target_state, occlusion=False, noise_sigma=0.0):
        """
        target_state: dict with 'x', 'y', 'w', 'h', 'vx', 'vy', 'ax', 'ay'
        Returns: (frame: np.ndarray, ground_truth: dict)
        """
        self.frame_count += 1
        # Dark gray/sky background
        frame = np.full((self.height, self.width, 3), 40, dtype=np.uint8)
        
        # Add background grid lines for visual texture
        for y in range(0, self.height, 40):
            cv2.line(frame, (0, y), (self.width, y), (50, 50, 50), 1)
        for x in range(0, self.width, 40):
            cv2.line(frame, (x, 0), (x, self.height), (50, 50, 50), 1)
            
        gt = {
            "frame_id": self.frame_count,
            "timestamp": self.frame_count * self.dt,
            "x": target_state["x"],
            "y": target_state["y"],
            "w": target_state["w"],
            "h": target_state["h"],
            "vx": target_state["vx"],
            "vy": target_state["vy"],
            "ax": target_state.get("ax", 0.0),
            "ay": target_state.get("ay", 0.0),
            "occluded": occlusion
        }
        
        if not occlusion:
            x1 = int(target_state["x"] - target_state["w"] / 2)
            y1 = int(target_state["y"] - target_state["h"] / 2)
            x2 = int(target_state["x"] + target_state["w"] / 2)
            y2 = int(target_state["y"] + target_state["h"] / 2)
            
            # Draw synthetic drone body (quadcopter cross shape + central fuselage)
            cx, cy = int(target_state["x"]), int(target_state["y"])
            cv2.line(frame, (x1, y1), (x2, y2), (200, 200, 200), 3)
            cv2.line(frame, (x1, y2), (x2, y1), (200, 200, 200), 3)
            cv2.circle(frame, (cx, cy), int(target_state["w"] * 0.2), (255, 255, 255), -1)
            # Rotors at 4 corners
            cv2.circle(frame, (x1, y1), 4, (0, 255, 255), -1)
            cv2.circle(frame, (x2, y1), 4, (0, 255, 255), -1)
            cv2.circle(frame, (x1, y2), 4, (0, 255, 255), -1)
            cv2.circle(frame, (x2, y2), 4, (0, 255, 255), -1)
            
        if noise_sigma > 0:
            noise = np.random.normal(0, noise_sigma, frame.shape).astype(np.int16)
            frame = np.clip(frame.astype(np.int16) + noise, 0, 255).astype(np.uint8)
            
        return frame, gt

class KalmanFilter6D:
    """6-State Kalman Filter: [x, y, vx, vy, ax, ay]"""
    def __init__(self, dt=1.0/30.0):
        self.dt = dt
        # cv2.KalmanFilter(dynamParams=6, measureParams=2)
        self.kf = cv2.KalmanFilter(6, 2)
        self.update_dt(dt)
        
        # Measurement matrix H: measures [x, y]
        self.kf.measurementMatrix = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0]
        ], dtype=np.float32)
        
        # Measurement noise covariance R
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 4.0 # 2 pixel std
        
        # Process noise covariance Q
        self.kf.processNoiseCov = np.eye(6, dtype=np.float32) * 1.0
        self.kf.errorCovPost = np.eye(6, dtype=np.float32) * 10.0
        self.initialized = False

    def update_dt(self, dt):
        self.dt = dt
        dt2 = 0.5 * dt * dt
        # State transition matrix F
        F = np.array([
            [1, 0, dt, 0, dt2, 0],
            [0, 1, 0, dt, 0, dt2],
            [0, 0, 1, 0, dt, 0],
            [0, 0, 0, 1, 0, dt],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ], dtype=np.float32)
        self.kf.transitionMatrix = F

    def init_state(self, x, y):
        self.kf.statePost = np.array([[x], [y], [0], [0], [0], [0]], dtype=np.float32)
        self.initialized = True

    def predict(self):
        return self.kf.predict()

    def update(self, x, y):
        meas = np.array([[np.float32(x)], [np.float32(y)]])
        return self.kf.correct(meas)

    def predict_future(self, dt_future):
        """Predicts position dt_future seconds into future."""
        current_state = self.kf.statePost.copy()
        dt2 = 0.5 * dt_future * dt_future
        F_future = np.array([
            [1, 0, dt_future, 0, dt2, 0],
            [0, 1, 0, dt_future, 0, dt2],
            [0, 0, 1, 0, dt_future, 0],
            [0, 0, 0, 1, 0, dt_future],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ], dtype=np.float32)
        future_state = np.dot(F_future, current_state)
        return future_state[0, 0], future_state[1, 0]

def test_synthetic_tracking():
    print("--- SYNTHETIC VIDEO & KALMAN 6D TRACKING TEST ---")
    gen = SyntheticVideoGenerator(640, 480, fps=30)
    kf = KalmanFilter6D(dt=1.0/30.0)
    
    # Target starts at (50, 100), moving at vx=120 px/s, vy=40 px/s, ax=10 px/s^2
    target = {"x": 50.0, "y": 100.0, "w": 40.0, "h": 40.0, "vx": 120.0, "vy": 40.0, "ax": 10.0, "ay": 0.0}
    
    kf.init_state(target["x"], target["y"])
    
    errors = []
    future_errors = []
    
    # 60 frames (2 seconds)
    # Occlusion occurs between frame 30 and 40 (10 frames dropout)
    for frame_idx in range(1, 61):
        dt = 1.0 / 30.0
        # Update ground truth kinematics
        target["vx"] += target["ax"] * dt
        target["vy"] += target["ay"] * dt
        target["x"] += target["vx"] * dt
        target["y"] += target["vy"] * dt
        
        is_occluded = (30 <= frame_idx <= 40)
        frame, gt = gen.generate_frame(target, occlusion=is_occluded)
        
        # Kalman Predict
        pred = kf.predict()
        
        if not is_occluded:
            # Measurement with 1.0 px noise
            meas_x = gt["x"] + np.random.normal(0, 1.0)
            meas_y = gt["y"] + np.random.normal(0, 1.0)
            corrected = kf.update(meas_x, meas_y)
            est_x, est_y = corrected[0, 0], corrected[1, 0]
        else:
            # Blind extrapolation using Kalman predict state
            est_x, est_y = pred[0, 0], pred[1, 0]
            
        # Error
        err = math.hypot(est_x - gt["x"], est_y - gt["y"])
        errors.append(err)
        
        # Predict 0.5s into future
        fx, fy = kf.predict_future(0.5)
        # GT future:
        gt_fx = gt["x"] + gt["vx"] * 0.5 + 0.5 * gt["ax"] * (0.5**2)
        gt_fy = gt["y"] + gt["vy"] * 0.5 + 0.5 * gt["ay"] * (0.5**2)
        future_err = math.hypot(fx - gt_fx, fy - gt_fy)
        future_errors.append(future_err)
        
        if frame_idx in [15, 30, 35, 40, 50]:
            tag = "OCCLUDED" if is_occluded else "TRACKING"
            print(f"Frame {frame_idx:02d} [{tag}]: GT=({gt['x']:.1f}, {gt['y']:.1f}), Est=({est_x:.1f}, {est_y:.1f}), PosErr={err:.2f}px, 0.5s_PredErr={future_err:.2f}px")

    mean_err = np.mean(errors)
    max_err = np.max(errors)
    occlusion_errs = errors[29:40]
    print(f"\nResults: Mean Error = {mean_err:.2f} px, Max Error = {max_err:.2f} px")
    print(f"Max Error during 10-frame Occlusion = {np.max(occlusion_errs):.2f} px")

if __name__ == '__main__':
    test_synthetic_tracking()
