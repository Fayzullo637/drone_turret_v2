# DISPATCH — 2026-09-02T02:12:02Z

## 2026-09-02T02:12:02Z
You are the Ballistics & Intercept Worker for drone_turret_v2 (Milestone M3).

Authoritative requirements file: C:\Users\User\teamwork_projects\drone_turret_v2\.agents\ORIGINAL_REQUEST.md
Scope document: C:\Users\User\teamwork_projects\drone_turret_v2\PROJECT.md
Spec reference: C:\Users\User\teamwork_projects\drone_turret_v2\.agents\survey_spec_explorer\handoff.md
Your metadata directory: C:\Users\User\teamwork_projects\drone_turret_v2\.agents\worker_m3_ballistics

MANDATORY INTEGRITY WARNING:
DO NOT CHEAT. All implementations must be genuine. DO NOT hardcode test results, create dummy/facade implementations, or circumvent the intended task. A teamwork_preview_auditor will independently verify your work. Integrity violations WILL be detected and your work WILL be rejected.

EXCLUSIVE WRITE OWNERSHIP:
You own `drone_turret/ballistics/` exclusively.

MISSION:
1. Implement `drone_turret/ballistics/__init__.py`.
2. Implement `drone_turret/ballistics/calculator.py`:
   - 4th-order Runge-Kutta (RK4) numerical integrator for net projectile 3D flight trajectory with gravity ($g=9.81\text{ m/s}^2$) and aerodynamic drag:
     $\frac{d\vec{v}_p}{dt} = -\frac{1}{2m} \rho C_d(t) A(t) \|\vec{v}_p\| \vec{v}_p + \vec{g}$.
   - Note: Implement using pure NumPy/Python (do not rely on scipy).
   - Dynamic net expansion model: $C_d(t) \in [1.1, 1.5]$ (or canister $0.45 \to 1.35$) and $A(t)$ expanding over time constant $\tau_{deploy}$.
   - Newton-Raphson root-finding algorithm: solve for intercept time $t_{int}$ where net flight distance matches projected drone 3D position $\vec{p}_d(t_{int})$.
   - Calculate lead aiming angles for pan servo $[0, 180]^\circ$ and tilt servo $[0, 180]^\circ$ with vertical gravity + drag drop compensation $\Delta Y_{drop}(t_{int})$.
   - Project 3D lead intercept point back onto 2D camera image coordinates `(x_lead, y_lead)` for tactical HUD crosshair overlay.
   - Benchmark validation: At 50m range with 60-80 m/s muzzle velocity, drone at 200 km/h (55.56 m/s) moves ~56m. Gravity drop at 50m: ~0.34m (vacuum/high speed) to ~2-3m.
   - Full parameter configurability: muzzle velocity $v_0$, projectile mass $m$, drag coefficient $C_d$, net area $A$, air density $\rho$.
3. Verify your implementation against analytical benchmarks, and write your report to `C:\Users\User\teamwork_projects\drone_turret_v2\.agents\worker_m3_ballistics\handoff.md`.

Send message back when complete.
