"""Pure navigation math (no ROS dependencies) so it can be unit-tested."""
import math


def compute_target(x0, y0, delta_x, delta_y):
    """Variant formula: target = start - delta."""
    return x0 - delta_x, y0 - delta_y


def distance_and_direction(x, y, x_target, y_target):
    """Return (distance, unit_x, unit_y) from the current point to the target."""
    dx = x_target - x
    dy = y_target - y
    dist = math.hypot(dx, dy)
    if dist < 1e-9:
        return 0.0, 0.0, 0.0
    return dist, dx / dist, dy / dist


def speed_profile(dist, max_speed, gain, min_speed):
    """Proportional speed with saturation: slows down while approaching."""
    return max(min(max_speed, gain * dist), min_speed)


def position_error(x, y, x_target, y_target):
    """e = sqrt((x - x_t)^2 + (y - y_t)^2)  (formula from section 2.8)."""
    return math.hypot(x - x_target, y - y_target)
