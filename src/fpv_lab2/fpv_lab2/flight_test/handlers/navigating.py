import time
from typing import TYPE_CHECKING

from ..flight_state import FlightState
from ..navigation import distance_and_direction, position_error, speed_profile
from .handler import FlightStateHandler

if TYPE_CHECKING:
    from ..flight_test_node import FlightTestNode

# The drone must stay inside the tolerance for this long (s) before the
# target is considered reached (filters out a fly-through and the inertia).
SETTLE_TIME = 2.0
LOG_PERIOD = 1.0


class NavigatingHandler(FlightStateHandler):
    """Flight to (x_target, y_target) with a proportional velocity law.

    v = clamp(K * distance, v_min, v_max), direction = unit vector to target.
    Altitude is held with a small proportional vertical velocity.
    """

    def __init__(self):
        self._last_log = 0.0
        self._hold_until = None

    def handle(self, node: "FlightTestNode") -> None:
        if node.status is None or not node.status.armed:
            node.fail("Drone disarmed unexpectedly during navigation")
            return

        # After the result is logged, hold position for a while (screenshot time)
        if self._hold_until is not None:
            node.publish_velocity(0.0, 0.0, 0.0)
            if time.monotonic() >= self._hold_until:
                self._hold_until = None
                node.get_logger().info("Hold at target finished, landing")
                node.state = FlightState.LANDING
            return

        if node.odometry is None or node.target is None:
            return

        now = time.monotonic()
        if node.nav_started_at is None:
            node.nav_started_at = now
            node.get_logger().info(
                "Navigation started, target "
                f"({node.target[0]:.3f}, {node.target[1]:.3f})"
            )

        p = node.odometry.pose.pose.position
        x_t, y_t = node.target
        tolerance = node.get_parameter("tolerance").value
        v_max = node.get_parameter("max_speed").value
        gain = node.get_parameter("speed_gain").value
        v_min = node.get_parameter("min_speed").value
        timeout = node.get_parameter("nav_timeout").value

        dist, ux, uy = distance_and_direction(p.x, p.y, x_t, y_t)
        # Vertical velocity is kept at zero: ArduPilot holds the altitude in
        # GUIDED velocity control (a proportional z-term diverged because of
        # the sign convention of the DDS velocity command).
        vz = 0.0

        if dist <= tolerance:
            # Inside the tolerance: stop the horizontal motion.
            node.publish_velocity(0.0, 0.0, vz)
            if node.nav_arrived_since is None:
                node.nav_arrived_since = now
            if now - node.nav_arrived_since >= SETTLE_TIME:
                self._finish(node, p, reached=True)
                self._hold_until = now + node.get_parameter("target_hold").value
            return

        node.nav_arrived_since = None

        if now - node.nav_started_at > timeout:
            node.publish_velocity(0.0, 0.0, 0.0)
            node.get_logger().warning(
                f"Navigation timeout ({timeout:.0f} s), landing here")
            self._finish(node, p, reached=False)
            self._hold_until = now + node.get_parameter("target_hold").value
            return

        speed = speed_profile(dist, v_max, gain, v_min)
        node.publish_velocity(speed * ux, speed * uy, vz)

        if now - self._last_log >= LOG_PERIOD:
            self._last_log = now
            node.get_logger().info(
                f"pos=({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) "
                f"dist={dist:.2f} m speed={speed:.2f} m/s"
            )

    @staticmethod
    def _finish(node: "FlightTestNode", p, reached: bool) -> None:
        x0, y0 = node.start_position
        x_t, y_t = node.target
        err = position_error(p.x, p.y, x_t, y_t)
        node.publish_velocity(0.0, 0.0, 0.0)
        node.get_logger().info(
            "Target point reached" if reached else "Target point NOT reached")
        node.get_logger().info(f"RESULT start    : x0={x0:.3f}, y0={y0:.3f}")
        node.get_logger().info(f"RESULT target   : x={x_t:.3f}, y={y_t:.3f}")
        node.get_logger().info(
            f"RESULT actual   : x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}")
        node.get_logger().info(f"RESULT error    : e={err:.3f} m")
        node.get_logger().info("Holding position at the target before landing")
