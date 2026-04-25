#!/usr/bin/env python3
"""Minimal keyboard teleop for the Dog2 bringup stack."""

from __future__ import annotations

import sys
import termios
import tty
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


USAGE = """
dog2 cmd_vel teleop
-------------------
w/s : forward/backward
a/d : yaw left/right
q/e : strafe left/right
space: stop
x : exit
"""


class CmdVelTeleop(Node):
    def __init__(self) -> None:
        super().__init__("dog2_cmd_vel_teleop")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("step_linear", 0.1)
        self.declare_parameter("step_angular", 0.3)

        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._step_linear = float(self.get_parameter("step_linear").value)
        self._step_angular = float(self.get_parameter("step_angular").value)

        self._publisher = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._twist = Twist()

        self.get_logger().info(USAGE)

    def _publish(self) -> None:
        self._publisher.publish(self._twist)

    def _stop(self) -> None:
        self._twist = Twist()
        self._publish()

    @staticmethod
    def _get_key() -> str:
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def run(self) -> None:
        try:
            while rclpy.ok():
                key = self._get_key()
                if key == "w":
                    self._twist.linear.x += self._step_linear
                elif key == "s":
                    self._twist.linear.x -= self._step_linear
                elif key == "q":
                    self._twist.linear.y += self._step_linear
                elif key == "e":
                    self._twist.linear.y -= self._step_linear
                elif key == "a":
                    self._twist.angular.z += self._step_angular
                elif key == "d":
                    self._twist.angular.z -= self._step_angular
                elif key == " ":
                    self._stop()
                    continue
                elif key == "x":
                    break
                self._publish()
        finally:
            self._stop()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = CmdVelTeleop()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
