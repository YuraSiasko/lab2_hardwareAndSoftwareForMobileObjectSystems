import math

from fpv_lab2.flight_test.navigation import (
    compute_target, distance_and_direction, position_error, speed_profile)


def test_variant_23_target():
    x_t, y_t = compute_target(0.0, 0.0, 5.2, 0.8)
    assert (x_t, y_t) == (-5.2, -0.8)


def test_distance_and_direction():
    d, ux, uy = distance_and_direction(0, 0, -5.2, -0.8)
    assert math.isclose(d, math.hypot(5.2, 0.8))
    assert math.isclose(math.hypot(ux, uy), 1.0)
    assert ux < 0 and uy < 0


def test_speed_profile_slows_down():
    far = speed_profile(5.0, 1.0, 0.6, 0.12)
    near = speed_profile(0.3, 1.0, 0.6, 0.12)
    assert far == 1.0 and 0.12 <= near < far


def test_error_formula():
    assert math.isclose(position_error(3, 4, 0, 0), 5.0)
