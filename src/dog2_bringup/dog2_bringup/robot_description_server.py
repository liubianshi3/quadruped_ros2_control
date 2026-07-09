#!/usr/bin/env python3
"""Tiny parameter server for Gazebo's gz_ros2_control URDF lookup."""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node


class RobotDescriptionServer(Node):
    def __init__(self) -> None:
        super().__init__(
            "gz_robot_description_server",
            automatically_declare_parameters_from_overrides=True,
        )
        description = self.get_parameter("robot_description").value
        if not isinstance(description, str) or not description:
            raise RuntimeError("robot_description parameter is required")
        self.get_logger().info(
            f"Serving robot_description parameter ({len(description)} chars)"
        )


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = RobotDescriptionServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
