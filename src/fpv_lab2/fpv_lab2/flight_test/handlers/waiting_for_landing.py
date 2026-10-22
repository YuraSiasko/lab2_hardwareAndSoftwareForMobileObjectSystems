from typing import TYPE_CHECKING

from ..flight_state import FlightState
from .handler import FlightStateHandler

if TYPE_CHECKING:
    from ..flight_test_node import FlightTestNode


class WaitingForLandingHandler(FlightStateHandler):

    def handle(self, node: "FlightTestNode") -> None:
        if node.status is None:
            return

        if not node.status.flying:
            node.get_logger().info("Landing completed")

            if node.odometry is not None:
                p = node.odometry.pose.pose.position
                node.get_logger().info(
                    f"Final Gazebo position: x={p.x:.3f}, y={p.y:.3f}, "
                    f"altitude={p.z:.2f} m"
                )

            node.state = FlightState.FINISHED
            node.get_logger().info("Flight test completed successfully")
