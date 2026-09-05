import math
import numpy as np

def solve_intercept_newton_raphson(
    target_pos,       # np.array([x0, y0, z0])
    target_vel,       # np.array([vx, vy, vz])
    target_acc,       # np.array([ax, ay, az])
    muzzle_velocity,  # float, e.g. 80.0 m/s
    drag_coeff=0.4,
    area=0.005,
    mass=0.8,
    rho=1.225,
    max_iter=20,
    tol=1e-3
):
    """
    Solves for intercept time t_int and required projectile aim vector.
    Target position at time t: r_t(t) = target_pos + target_vel * t + 0.5 * target_acc * t^2
    Projectile travel distance function R_p(t) accounting for 1D drag along path.
    """
    # Projectile distance under quadratic drag:
    # dv/dt = -k * v^2 where k = 0.5 * rho * Cd * A / m
    k = 0.5 * rho * drag_coeff * area / mass
    
    def projectile_dist(t):
        if k < 1e-9:
            return muzzle_velocity * t
        # v(t) = v0 / (1 + k * v0 * t)
        # R(t) = (1/k) * ln(1 + k * v0 * t)
        return (1.0 / k) * math.log(1.0 + k * muzzle_velocity * t)
    
    def projectile_speed(t):
        if k < 1e-9:
            return muzzle_velocity
        return muzzle_velocity / (1.0 + k * muzzle_velocity * t)

    # Initial estimate: Euclidean distance / muzzle_velocity
    r0 = np.linalg.norm(target_pos)
    t = r0 / muzzle_velocity
    
    for i in range(max_iter):
        r_t = target_pos + target_vel * t + 0.5 * target_acc * (t**2)
        dist_t = np.linalg.norm(r_t)
        dist_p = projectile_dist(t)
        
        f = dist_p - dist_t
        if abs(f) < tol:
            # Found intercept time t
            # Target intercept position
            p_intercept = r_t
            
            # Gravity compensation: projectile drops by RK4/drag drop during time t
            # Drop in vacuum: 0.5 * g * t^2. Under drag, drop is slightly larger:
            g = 9.81
            gravity_drop = 0.5 * g * (t**2) # baseline
            aim_point = p_intercept.copy()
            aim_point[1] += gravity_drop # elevate aim point (+Y is UP)
            
            return {
                "converged": True,
                "iterations": i + 1,
                "t_intercept": t,
                "target_intercept_pos": p_intercept,
                "aim_point": aim_point,
                "distance": dist_t,
                "gravity_drop": gravity_drop
            }
        
        # Derivative f'(t) = v_p(t) - d/dt(||r_t(t)||)
        v_t_current = target_vel + target_acc * t
        dr_t_dt = np.dot(r_t, v_t_current) / (dist_t + 1e-9)
        f_prime = projectile_speed(t) - dr_t_dt
        
        if abs(f_prime) < 1e-6:
            break
        
        t_next = t - f / f_prime
        if t_next <= 0:
            t_next = t * 0.5
        t = t_next

    return {"converged": False, "iterations": max_iter, "t_intercept": t}

def test_solver():
    print("--- NEWTON-RAPHSON INTERCEPT SOLVER BENCHMARK ---")
    
    # Case 1: Stationary target at 50m
    res1 = solve_intercept_newton_raphson(
        np.array([0.0, 0.0, 50.0]),
        np.array([0.0, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.0]),
        muzzle_velocity=80.0
    )
    print(f"Case 1 (Stationary 50m): Converged={res1['converged']} in {res1['iterations']} iters, t={res1['t_intercept']:.4f}s, Aim drop={res1['gravity_drop']:.3f}m")

    # Case 2: Crossing target at 200 km/h (55.56 m/s) along X at 50m Z
    res2 = solve_intercept_newton_raphson(
        np.array([-20.0, 0.0, 50.0]),
        np.array([55.56, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.0]),
        muzzle_velocity=80.0
    )
    print(f"Case 2 (Crossing 200km/h): Converged={res2['converged']} in {res2['iterations']} iters, t={res2['t_intercept']:.4f}s, Lead point={res2['aim_point']}")

    # Case 3: Accelerating target (evasion: 10 m/s^2)
    res3 = solve_intercept_newton_raphson(
        np.array([0.0, 10.0, 40.0]),
        np.array([20.0, 5.0, 0.0]),
        np.array([5.0, 10.0, 0.0]),
        muzzle_velocity=80.0
    )
    print(f"Case 3 (Accelerating): Converged={res3['converged']} in {res3['iterations']} iters, t={res3['t_intercept']:.4f}s, Aim point={res3['aim_point']}")

if __name__ == '__main__':
    test_solver()
