#!/usr/bin/env python3
"""Minimal gait scheduler for phase-1/phase-2 system integration."""

from __future__ import annotations

from typing import List, Optional

import rclpy
from dog2_interfaces.msg import ContactPhase, GaitCommand
from geometry_msgs.msg import Twist
from rclpy.node import Node

LEG_NAMES = ["lf", "lh", "rh", "rf"]


def compute_phase_array(gait: str, phase: float, moving: bool) -> List[int]:
    if not moving:
        return [ContactPhase.STANCE for _ in LEG_NAMES]
    if gait == "trot":
        return [
            ContactPhase.STANCE if phase < 0.5 else ContactPhase.SWING,
            ContactPhase.SWING if phase < 0.5 else ContactPhase.STANCE,
            ContactPhase.STANCE if phase < 0.5 else ContactPhase.SWING,
            ContactPhase.SWING if phase < 0.5 else ContactPhase.STANCE,
        ]
    return [ContactPhase.STANCE, ContactPhase.STANCE, ContactPhase.STANCE, ContactPhase.SWING]


class GaitSchedulerNode(Node):
    def __init__(self) -> None:
        super().__init__("dog2_gait_scheduler")

        self.declare_parameter("gait", "trot")
        self.declare_parameter("cycle_time", 0.8)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("contact_phase_topic", "/dog2/gait/contact_phase")
        self.declare_parameter("gait_command_topic", "/dog2/gait/command")

        self._gait = str(self.get_parameter("gait").value)
        self._cycle_time = max(0.1, float(self.get_parameter("cycle_time").value))
        self._publish_rate = max(1.0, float(self.get_parameter("publish_rate").value))
        self._elapsed = 0.0
        self._moving = False
        self._last_cmd = Twist()

        self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self._on_cmd_vel,
            20,
        )
        self._contact_pub = self.create_publisher(
            ContactPhase,
            str(self.get_parameter("contact_phase_topic").value),
            10,
        )
        self._gait_cmd_pub = self.create_publisher(
            GaitCommand,
            str(self.get_parameter("gait_command_topic").value),
            10,
        )
        self.create_timer(1.0 / self._publish_rate, self._on_timer)

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd = msg
        self._moving = any(
            abs(value) > 1e-3
            for value in [msg.linear.x, msg.linear.y, msg.angular.z]
        )

    def _on_timer(self) -> None:
        self._elapsed = (self._elapsed + 1.0 / self._publish_rate) % self._cycle_time
        phase = self._elapsed / self._cycle_time

        contact_msg = ContactPhase()
        contact_msg.header.stamp = self.get_clock().now().to_msg()
        contact_msg.gait = self._gait
        contact_msg.leg_names = list(LEG_NAMES)
        contact_msg.phase = compute_phase_array(self._gait, phase, self._moving)
        contact_msg.cycle_time = float(self._cycle_time)

        gait_cmd_msg = GaitCommand()
        gait_cmd_msg.header = contact_msg.header
        gait_cmd_msg.gait = self._gait
        gait_cmd_msg.linear_x = float(self._last_cmd.linear.x)
        gait_cmd_msg.linear_y = float(self._last_cmd.linear.y)
        gait_cmd_msg.angular_z = float(self._last_cmd.angular.z)

        self._contact_pub.publish(contact_msg)
        self._gait_cmd_pub.publish(gait_cmd_msg)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = GaitSchedulerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
