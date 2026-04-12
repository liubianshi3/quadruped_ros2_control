import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.srv import ControlWorld
from std_msgs.msg import Bool


class GzStartupGate(Node):
    """Wait until controllers and the spider startup prearm are ready, then unpause Gazebo."""

    def __init__(self):
        super().__init__("gz_startup_gate")

        self.declare_parameter("controller_manager_name", "/controller_manager")
        self.declare_parameter(
            "required_controllers",
            ["joint_state_broadcaster", "joint_trajectory_controller", "rail_position_controller"],
        )
        self.declare_parameter("ready_topic", "/spider_startup_ready")
        self.declare_parameter("world_name", "empty")
        self.declare_parameter("timeout_s", 90.0)
        self.declare_parameter("poll_period_s", 0.2)
        self.declare_parameter("settle_time_s", 0.2)
        self.declare_parameter("unpause_prepare_topic", "/gazebo_unpause_prepare")
        self.declare_parameter("unpaused_topic", "/gazebo_unpaused")

        controller_manager_name = str(self.get_parameter("controller_manager_name").value).strip() or "/controller_manager"
        if not controller_manager_name.startswith("/"):
            controller_manager_name = f"/{controller_manager_name}"
        self._required_controllers = [str(name) for name in self.get_parameter("required_controllers").value]
        ready_topic = str(self.get_parameter("ready_topic").value).strip() or "/spider_startup_ready"
        world_name = str(self.get_parameter("world_name").value).strip().strip("/") or "empty"

        self._list_client = self.create_client(ListControllers, f"{controller_manager_name}/list_controllers")
        self._control_service_name = f"/world/{world_name}/control"
        self._control_client = self.create_client(ControlWorld, self._control_service_name)

        ready_qos = QoSProfile(depth=1)
        ready_qos.reliability = ReliabilityPolicy.RELIABLE
        ready_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._ready_sub = self.create_subscription(Bool, ready_topic, self._on_ready_msg, ready_qos)
        unpause_prepare_topic = str(self.get_parameter("unpause_prepare_topic").value).strip() or "/gazebo_unpause_prepare"
        unpaused_topic = str(self.get_parameter("unpaused_topic").value).strip() or "/gazebo_unpaused"
        self._unpause_prepare_pub = self.create_publisher(Bool, unpause_prepare_topic, ready_qos)
        self._unpaused_pub = self.create_publisher(Bool, unpaused_topic, ready_qos)

        self._startup_ready = False
        self._controllers_ready = False
        self._all_ready_since = None
        self._start_wall_time = time.monotonic()
        self._list_future = None
        self._control_future = None
        self._unpause_prepare_published = False
        self._unpaused_published = False

        poll_period_s = float(self.get_parameter("poll_period_s").value)
        self._timer = self.create_timer(
            max(0.05, poll_period_s),
            self._tick,
            clock=Clock(clock_type=ClockType.SYSTEM_TIME),
        )

        self.get_logger().info(
            "Startup gate waiting for controllers=%s, ready_topic=%s, world_control=%s"
            % (self._required_controllers, ready_topic, self._control_service_name)
        )

    def _on_ready_msg(self, msg: Bool) -> None:
        if not bool(getattr(msg, "data", False)):
            return
        if self._startup_ready:
            return
        self._startup_ready = True
        self.get_logger().info("Received spider startup ready signal.")

    def _tick(self) -> None:
        timeout_s = float(self.get_parameter("timeout_s").value)
        if time.monotonic() - self._start_wall_time > timeout_s:
            self.get_logger().error("Startup gate timed out before Gazebo could be unpaused.")
            rclpy.shutdown()
            return

        if not self._controllers_ready:
            self._request_controller_state()
            return

        if not self._startup_ready:
            return

        if self._all_ready_since is None:
            self._all_ready_since = time.monotonic()

        settle_time_s = max(0.0, float(self.get_parameter("settle_time_s").value))
        if time.monotonic() - self._all_ready_since < settle_time_s:
            return

        self._request_unpause()

    def _request_controller_state(self) -> None:
        if self._list_future is not None:
            return
        if not self._list_client.wait_for_service(timeout_sec=0.0):
            return

        self._list_future = self._list_client.call_async(ListControllers.Request())
        self._list_future.add_done_callback(self._on_list_controllers_response)

    def _on_list_controllers_response(self, future) -> None:
        self._list_future = None
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"Failed to query controller states: {exc}")
            return

        controller_states = {controller.name: controller.state.lower() for controller in response.controller}
        missing = [
            name for name in self._required_controllers if controller_states.get(name, "") != "active"
        ]
        if missing:
            self._controllers_ready = False
            return

        if not self._controllers_ready:
            self.get_logger().info("All required controllers are active.")
        self._controllers_ready = True

    def _request_unpause(self) -> None:
        if self._control_future is not None:
            return
        if not self._control_client.wait_for_service(timeout_sec=0.0):
            return

        self._publish_bool_once(self._unpause_prepare_pub, "_unpause_prepare_published")
        request = ControlWorld.Request()
        request.world_control.pause = False
        self._control_future = self._control_client.call_async(request)
        self._control_future.add_done_callback(self._on_unpause_response)

    def _on_unpause_response(self, future) -> None:
        self._control_future = None
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"Failed to unpause Gazebo: {exc}")
            return

        if not bool(getattr(response, "success", False)):
            self.get_logger().warn("Gazebo unpause request was rejected; retrying.")
            return

        self._publish_bool_once(self._unpaused_pub, "_unpaused_published")
        self.get_logger().info("Gazebo unpaused after controller and startup readiness checks.")
        rclpy.shutdown()

    def _publish_bool_once(self, publisher, flag_attr: str) -> None:
        if bool(getattr(self, flag_attr, False)):
            return
        msg = Bool()
        msg.data = True
        publisher.publish(msg)
        setattr(self, flag_attr, True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GzStartupGate()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
