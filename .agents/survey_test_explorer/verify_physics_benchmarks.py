import math
import numpy as np

def rk4_step(state, dt, mass=0.8, cd=0.5, area=0.005, rho=1.225, g=9.81):
    def deriv(s):
        x, y, z, vx, vy, vz = s
        v = math.sqrt(vx**2 + vy**2 + vz**2)
        drag_const = 0.5 * rho * cd * area / mass
        ax = -drag_const * v * vx
        ay = -g - drag_const * v * vy
        az = -drag_const * v * vz
        return np.array([vx, vy, vz, ax, ay, az], dtype=float)
    
    k1 = deriv(state)
    k2 = deriv(state + 0.5 * dt * k1)
    k3 = deriv(state + 0.5 * dt * k2)
    k4 = deriv(state + dt * k3)
    return state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

def simulate_ballistics():
    configs = [
        ("Vacuum (v0=60m/s)", 60.0, 0.8, 0.0, 0.0),
        ("Vacuum (v0=80m/s)", 80.0, 0.8, 0.0, 0.0),
        ("Canister Flight (Cd=0.4, A=0.005, m=0.8kg, v0=60m/s)", 60.0, 0.8, 0.4, 0.005),
        ("Canister Flight (Cd=0.4, A=0.005, m=0.8kg, v0=80m/s)", 80.0, 0.8, 0.4, 0.005),
        ("Expanded Net (Cd=1.2, A_eff=0.015, m=0.6kg, v0=80m/s)", 80.0, 0.6, 1.2, 0.015),
    ]

    print("--- 50M RANGE BALLISTIC BENCHMARKS ---")
    for name, v0, m, cd, area in configs:
        s = np.array([0.0, 0.0, 0.0, v0, 0.0, 0.0], dtype=float)
        dt = 0.001
        t = 0.0
        while s[0] < 50.0 and t < 5.0:
            s = rk4_step(s, dt, mass=m, cd=cd, area=area)
            t += dt
        
        # Lead point for target moving at 200 km/h (55.56 m/s) orthogonal
        v_target = 200.0 / 3.6 # 55.556 m/s
        lead_x = v_target * t
        
        print(f"[{name}]")
        print(f"  Time of flight: {t:.4f} s")
        print(f"  Gravity drop: {s[1]:.4f} m (absolute drop: {abs(s[1]):.4f} m)")
        print(f"  Remaining velocity: {s[3]:.2f} m/s (lost {((v0 - s[3])/v0)*100:.1f}%)")
        print(f"  Drone lead distance (v=200 km/h): {lead_x:.2f} m\n")

if __name__ == '__main__':
    simulate_ballistics()
