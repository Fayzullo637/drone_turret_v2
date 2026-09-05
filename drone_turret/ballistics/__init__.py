"""
Ballistics package for drone_turret_v2.

Provides:
- BallisticCalculator: RK4 trajectory integration and Newton-Raphson intercept solver.
- BallisticConfig: Configuration dataclass for physical and numerical parameters.
- TargetState: Kinematic state vector dataclass for tracked targets.
- InterceptSolution: Computed firing and intercept lead solution dataclass.
- TrajectoryPoint: Trajectory state snapshot dataclass.
"""

from drone_turret.ballistics.calculator import (
    BallisticCalculator,
    BallisticConfig,
    InterceptSolution,
    TargetState,
    TrajectoryPoint,
)

__all__ = [
    "BallisticCalculator",
    "BallisticConfig",
    "InterceptSolution",
    "TargetState",
    "TrajectoryPoint",
]
