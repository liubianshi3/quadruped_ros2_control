"""Continuously command the rail position controller to hold lock targets.

The 4 prismatic rail joints are claimed by `rail_position_controller`
(position command interface). gz_ros2_control turns the position target into
a stiff 1kHz servo, so streaming a constant target effectively welds the
rails during normal locomotion. Crossing work can later reuse this node (or
publish to the same topic) to move the rails on purpose.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class RailLockCommander(Node):
    def __init__(self) -> None:
        super().__init__("rail_lock_commander")

        self.declare_parameter("output_topic", "/rail_position_controller/commands")
        self.declare_parameter("rail_targets", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("publish_rate_hz", 20.0)

        self._output_topic = str(self.get_parameter("output_topic").value)
        targets = [float(v) for v in self.get_parameter("rail_targets").value]
        if len(targets) != 4:
            raise ValueError("rail_targets must be [lf, lh, rh, rf]")
        self._targets = targets
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self._pub = self.create_publisher(Float64MultiArray, self._output_topic, 10)
        self.create_timer(max(1.0 / publish_rate_hz, 0.005), self._on_timer)

        self.get_logger().info(
            "rail_lock_commander ready: topic='%s' targets=[%.3f, %.3f, %.3f, %.3f]"
            % (self._output_topic, *self._targets)
        )

    def _on_timer(self) -> None:
        msg = Float64MultiArray()
        msg.data = list(self._targets)
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = RailLockCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
