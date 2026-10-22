import time

from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from geographic_msgs.msg import GeoPoseStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

from ardupilot_msgs.msg import Status
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff

from .flight_state import FlightState
from .handlers import HANDLERS
from .navigation import compute_target


class FlightTestNode(Node):

    GUIDED_MODE = 4
    LAND_MODE = 9

    TAKEOFF_ALTITUDE = 2.0
    HOVER_DURATION = 20.0      # s (wall time) hover before the horizontal flight; sim runs slowly in WSL

    def __init__(self):
        super().__init__("flight_test")

        # Individual task: variant 23 (delta_x = 5.2 m, delta_y = 0.8 m)
        self.declare_parameter("delta_x", 5.2)
        self.declare_parameter("delta_y", 0.8)
        self.declare_parameter("max_speed", 0.4)       # m/s
        self.declare_parameter("speed_gain", 0.4)      # 1/s
        self.declare_parameter("min_speed", 0.10)      # m/s
        self.declare_parameter("tolerance", 0.20)      # m
        self.declare_parameter("nav_timeout", 600.0)   # s (wall time)
        self.declare_parameter("target_hold", 10.0)    # s, hover at target before landing
        self.declare_parameter("cmd_vel_topic", "/ap/v1/cmd_vel")
        self.declare_parameter("cmd_vel_frame", "map")   # ArduPilot DDS accepts only "map" (ENU) or "base_link"

        self.delta_x = self.get_parameter("delta_x").value
        self.delta_y = self.get_parameter("delta_y").value

        self.state = FlightState.WAITING_FOR_SYSTEM
        self.request_in_progress = False

        self.status = None
        self.odometry = None
        self.hover_started_at = None
        self.geopose_first_at = None   # first filtered GeoPose = EKF has a position

        self.start_position = None     # (x0, y0) from the first odometry
        self.target = None             # (x_target, y_target)
        self.nav_started_at = None
        self.nav_arrived_since = None

        self._handlers = HANDLERS

        self.status_subscription = self.create_subscription(
            Status, "/ap/v1/status", self.status_callback, 10)

        self.odometry_subscription = self.create_subscription(
            Odometry, "/odometry", self.odometry_callback, 10)

        self.geopose_subscription = self.create_subscription(
            GeoPoseStamped, "/ap/v1/geopose/filtered", self.geopose_callback, 10)

        self.cmd_vel_publisher = self.create_publisher(
            TwistStamped, self.get_parameter("cmd_vel_topic").value, 10)

        self.prearm_client = self.create_client(Trigger, "/ap/v1/prearm_check")
        self.mode_client = self.create_client(ModeSwitch, "/ap/v1/mode_switch")
        self.arm_client = self.create_client(ArmMotors, "/ap/v1/arm_motors")
        self.takeoff_client = self.create_client(
            Takeoff, "/ap/v1/experimental/takeoff")

        self.timer = self.create_timer(0.2, self.control_loop)

        self.get_logger().info("Flight test node started")
        self.get_logger().info("Waiting for ArduPilot and ROS services...")

    def status_callback(self, message):
        self.status = message

    def geopose_callback(self, message):
        if self.geopose_first_at is None:
            self.geopose_first_at = time.monotonic()
            self.get_logger().info("EKF position estimate is available")

    def odometry_callback(self, message):
        self.odometry = message
        if self.start_position is None:
            p = message.pose.pose.position
            self.start_position = (p.x, p.y)
            self.target = compute_target(p.x, p.y, self.delta_x, self.delta_y)
            self.get_logger().info(
                f"Start position: x0={p.x:.3f}, y0={p.y:.3f}")
            self.get_logger().info(
                f"Target point (variant 23): x={self.target[0]:.3f}, "
                f"y={self.target[1]:.3f}  "
                f"(delta_x={self.delta_x}, delta_y={self.delta_y})")

    def services_ready(self):
        return (
            self.prearm_client.service_is_ready()
            and self.mode_client.service_is_ready()
            and self.arm_client.service_is_ready()
            and self.takeoff_client.service_is_ready()
        )

    def control_loop(self):
        handler = self._handlers.get(self.state)
        if handler is not None:
            handler.handle(self)

    def publish_velocity(self, vx, vy, vz=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("cmd_vel_frame").value
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        self.cmd_vel_publisher.publish(msg)

    def call_mode_switch(self, mode, callback):
        self.request_in_progress = True

        request = ModeSwitch.Request()
        request.mode = mode

        future = self.mode_client.call_async(request)
        future.add_done_callback(callback)

        self.get_logger().info(f"Requesting flight mode {mode}")

    def fail(self, message):
        self.get_logger().error(message)
        self.state = FlightState.ERROR
