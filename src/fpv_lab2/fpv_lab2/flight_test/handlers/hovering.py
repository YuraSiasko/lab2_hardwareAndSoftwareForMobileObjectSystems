import time
from typing import TYPE_CHECKING

from ..flight_state import FlightState
from .handler import FlightStateHandler

if TYPE_CHECKING:
    from ..flight_test_node import FlightTestNode


class HoveringHandler(FlightStateHandler):
    """Waits for the takeoff altitude, then hovers for HOVER_DURATION."""

    STATUS_GRACE = 8.0       # s, status topic may lag right after takeoff
    CLIMB_TIMEOUT = 300.0    # s, the simulation runs slower than real time
    ALTITUDE_FRACTION = 0.9  # fraction of the takeoff altitude

    def __init__(self) -> None:
        self._reached_at = None
        self._last_log = 0.0

    def handle(self, node: "FlightTestNode") -> None:
        now = time.monotonic()
        since_takeoff = now - node.hover_started_at

        if since_takeoff > self.STATUS_GRACE and (
            node.status is None or not node.status.armed
        ):
            node.fail("Drone disarmed unexpectedly during flight")
            return

        altitude = (
            node.odometry.pose.pose.position.z
            if node.odometry is not None else 0.0
        )

        if self._reached_at is None:
            if altitude >= self.ALTITUDE_FRACTION * node.TAKEOFF_ALTITUDE:
                self._reached_at = now
                node.get_logger().info(
                    f"Takeoff altitude reached: {altitude:.2f} m"
                )
            elif since_takeoff > self.CLIMB_TIMEOUT:
                node.fail("Takeoff altitude was not reached in time")
            elif now - self._last_log > 5.0:
                self._last_log = now
                node.get_logger().info(f"Climbing, altitude {altitude:.2f} m")
            return

        if now - self._reached_at >= node.HOVER_DURATION:
            node.get_logger().info("Hover completed, flying to the target point")
            node.state = FlightState.NAVIGATING
