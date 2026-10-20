"""Low-speed Pure Pursuit; see docs/provenance.md for the MP2 reference.

Input points are ordered ahead of the rear axle in its instantaneous planar
frame (x forward, y left). This module neither finds lanes nor transforms TF.
"""

from dataclasses import dataclass
import math
from typing import Sequence, Tuple

Point = Tuple[float, float]


@dataclass(frozen=True)
class ControlConfig:
    wheelbase_m: float = 1.75
    lookahead_m: float = 2.0
    max_steering_rad: float = 0.45
    cruise_speed_mps: float = 1.0
    max_lateral_accel_mps2: float = 0.4
    minimum_target_distance_m: float = 0.2

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if self.max_steering_rad >= math.pi / 2:
            raise ValueError("steering limit must be below pi/2")


@dataclass(frozen=True)
class ControlResult:
    steering_rad: float
    speed_mps: float
    target: Point
    curvature_per_m: float


def pure_pursuit(points: Sequence[Point], config: ControlConfig) -> ControlResult:
    """Track an already ordered, forward-only local path, or reject it.

    The local planner must trim passed waypoints; this controller does not pick
    an arbitrary closest point on a looping path. Duplicate points are allowed.
    All points must be finite. A path entirely behind or at the axle is invalid.
    """
    if len(points) < 2:
        raise ValueError("at least two local-path points are required")
    path = [(float(x), float(y)) for x, y in points]
    if any(not math.isfinite(v) for p in path for v in p):
        raise ValueError("path contains non-finite coordinates")
    if any(x < -1e-6 for x, _ in path):
        raise ValueError("path must be trimmed to the forward half-plane")

    target = path[-1]
    distance = 0.0
    has_segment = False
    for start, end in zip(path, path[1:]):
        segment = math.hypot(end[0] - start[0], end[1] - start[1])
        if segment <= 1e-9:
            continue
        has_segment = True
        if distance + segment >= config.lookahead_m:
            ratio = (config.lookahead_m - distance) / segment
            target = (start[0] + ratio * (end[0] - start[0]),
                      start[1] + ratio * (end[1] - start[1]))
            break
        distance += segment

    if not has_segment:
        raise ValueError("path has no nonzero segment")

    radius_sq = target[0] ** 2 + target[1] ** 2
    if target[0] <= 0 or radius_sq < config.minimum_target_distance_m ** 2:
        raise ValueError("no usable target in front of the vehicle")
    curvature = 2.0 * target[1] / radius_sq
    steering = math.atan(config.wheelbase_m * curvature)
    steering = max(-config.max_steering_rad,
                   min(config.max_steering_rad, steering))
    speed = config.cruise_speed_mps
    if abs(curvature) > 1e-9:
        speed = min(speed, math.sqrt(config.max_lateral_accel_mps2 / abs(curvature)))
    return ControlResult(steering, speed, target, curvature)
