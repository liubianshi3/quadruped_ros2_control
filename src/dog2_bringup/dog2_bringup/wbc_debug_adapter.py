#!/usr/bin/env python3
"""Adapt legacy WBC effort outputs into unified Dog2 debug topics."""

from __future__ import annotations

from typing import List, Optional

import rclpy
from dog2_interfaces.msg import WBCDebug
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class WbcDebugAdapter(Node):
    def __init__(self) -> None:
        super().__init__("dog2_wbc_debug_adapter")

        self.declare_parameter("torque_topic", "/dog2/wbc/joint_effort_command")
        self.declare_parameter("debug_topic", "/dog2/wbc/debug")
        self.declare_parameter("controller_mode", "research_wbc")

        self._controller_mode = str(self.get_parameter("controller_mode").value)
        self._last_effort_msg: Optional[Float64MultiArray] = None

        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("torque_topic").value),
            self._on_effort,
            20,
        )
        self._debug_pub = self.create_publisher(WBCDebug, str(self.get_parameter("debug_topic").value), 10)

    def _on_effort(self, msg: Float64MultiArray) -> None:
        self._last_effort_msg = msg
        debug_msg = WBCDebug()
        debug_msg.controller_mode = self._controller_mode
        debug_msg.solve_time_ms = 0.0
        debug_msg.success = True
        debug_msg.torque_norm = float(sum(abs(x) for x in msg.data))
        debug_msg.joint_effort_command = list(msg.data)
        debug_msg.active_tasks = [
            "base_attitude",
            "base_height",
            "contact_consistency",
            "swing_tracking",
        ]
        debug_msg.status_message = "legacy_wbc_adapter"
        self._debug_pub.publish(debug_msg)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = WbcDebugAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
