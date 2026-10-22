import time
from functools import partial
from typing import TYPE_CHECKING

from std_srvs.srv import Trigger

from ..flight_state import FlightState
from .handler import FlightStateHandler

if TYPE_CHECKING:
    from ..flight_test_node import FlightTestNode


class PrearmCheckHandler(FlightStateHandler):
    # The EKF needs some time after a fresh SITL start (slow simulation),
    # so a negative answer is retried for a while before the test fails.
    RETRY_TIMEOUT = 300.0
    RETRY_PERIOD = 3.0

    EKF_SETTLE = 45.0     # s (wall) after the first position estimate
    EKF_TIMEOUT = 60.0    # s (wall) after the pre-arm check passed

    def __init__(self) -> None:
        self._passed = False
        self._passed_at = 0.0
        self._last_log = 0.0
        self._first_try = None
        self._next_try = 0.0

    def handle(self, node: "FlightTestNode") -> None:
        now = time.monotonic()
        if self._first_try is None:
            self._first_try = now
        if self._passed:
            self._wait_for_ekf(node, now)
        elif not node.request_in_progress and now >= self._next_try:
            self.call_prearm_check(node)

    def _wait_for_ekf(self, node: "FlightTestNode", now: float) -> None:
        # Arming too early makes the EKF re-align its yaw in flight (the
        # slow simulation delays GPS), which crashed the drone. Wait for the
        # position estimate plus a settling time.
        got = node.geopose_first_at
        if got is not None and now - got >= self.EKF_SETTLE:
            node.state = FlightState.SETTING_GUIDED
        elif now - self._passed_at >= self.EKF_TIMEOUT:
            node.get_logger().warn("EKF wait timed out, continuing")
            node.state = FlightState.SETTING_GUIDED
        elif now - self._last_log > 10.0:
            self._last_log = now
            node.get_logger().info("Waiting for the EKF to settle before arming...")

    def call_prearm_check(self, node: "FlightTestNode") -> None:
        node.request_in_progress = True

        request = Trigger.Request()
        future = node.prearm_client.call_async(request)
        future.add_done_callback(partial(self.prearm_callback, node))

        node.get_logger().info("Running pre-arm check")

    def prearm_callback(self, node: "FlightTestNode", future) -> None:
        node.request_in_progress = False

        try:
            response = future.result()
        except Exception as error:
            node.fail(f"Pre-arm service failed: {error}")
            return

        if not response.success:
            if time.monotonic() - self._first_try < self.RETRY_TIMEOUT:
                node.get_logger().warn(
                    f"Not armable yet ({response.message}), retrying...")
                self._next_try = time.monotonic() + self.RETRY_PERIOD
                return
            node.fail(f"Vehicle is not armable: {response.message}")
            return

        node.get_logger().info(f"Pre-arm check passed: {response.message}")
        self._passed = True
        self._passed_at = time.monotonic()
