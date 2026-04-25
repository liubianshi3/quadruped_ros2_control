#!/usr/bin/env python3
"""Delayed one-shot publisher for the research crossing trigger."""

from __future__ import annotations

import math
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


class CrossingTrigger(Node):
    def __init__(self) -> None:
        super().__init__("dog2_crossing_trigger")

        self.declare_parameter("enabled", True)
        self.declare_parameter("delay_sec", 5.0)
        self.declare_parameter("topic", "/enable_crossing")
        self.declare_parameter("wait_for_stack_ready", True)
        self.declare_parameter("odom_topic", "/dog2/state_estimation/odom")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("ready_freshness_sec", 1.0)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("approach_cmd_vel_x", 0.0)
        self.declare_parameter("approach_cmd_vel_y", 0.0)
        self.declare_parameter("approach_cmd_vel_yaw", 0.0)
        self.declare_parameter("trigger_when_x_ge", float("nan"))
        self.declare_parameter("stop_cmd_vel_on_trigger", True)

        self._enabled = bool(self.get_parameter("enabled").value)
        self._delay_sec = max(0.0, float(self.get_parameter("delay_sec").value))
        self._topic = str(self.get_parameter("topic").value)
        self._wait_for_stack_ready = bool(self.get_parameter("wait_for_stack_ready").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._approach_cmd_vel_x = float(self.get_parameter("approach_cmd_vel_x").value)
        self._approach_cmd_vel_y = float(self.get_parameter("approach_cmd_vel_y").value)
        self._approach_cmd_vel_yaw = float(
            self.get_parameter("approach_cmd_vel_yaw").value
        )
        self._trigger_when_x_ge = float(self.get_parameter("trigger_when_x_ge").value)
        self._stop_cmd_vel_on_trigger = bool(
            self.get_parameter("stop_cmd_vel_on_trigger").value
        )
        self._ready_freshness_sec = max(
            0.1, float(self.get_parameter("ready_freshness_sec").value)
        )
        self._publisher = self.create_publisher(Bool, self._topic, 1)
        self._cmd_vel_pub = self.create_publisher(Twist, self._cmd_vel_topic, 1)
        self._fired = False
        self._started_ns = int(self.get_clock().now().nanoseconds)
        self._last_odom_ns: Optional[int] = None
        self._last_joint_state_ns: Optional[int] = None
        self._latest_odom_x: Optional[float] = None
        self._last_wait_log_ns = 0
        if self._wait_for_stack_ready or math.isfinite(self._trigger_when_x_ge):
            self._odom_sub = self.create_subscription(
                Odometry, self._odom_topic, self._on_odom, 10
            )
        else:
            self._odom_sub = None
        if self._wait_for_stack_ready:
            self._joint_state_sub = self.create_subscription(
                JointState, self._joint_state_topic, self._on_joint_state, 10
            )
        else:
            self._joint_state_sub = None
        self._timer = self.create_timer(0.1, self._on_timer)

        self.get_logger().info(
            "crossing_trigger ready: enabled=%s topic='%s' delay=%.2fs wait_for_stack_ready=%s "
            "odom='%s' joint_states='%s' approach_cmd_vel=[%.3f, %.3f, %.3f] trigger_when_x_ge=%s"
            % (
                self._enabled,
                self._topic,
                self._delay_sec,
                self._wait_for_stack_ready,
                self._odom_topic,
                self._joint_state_topic,
                self._approach_cmd_vel_x,
                self._approach_cmd_vel_y,
                self._approach_cmd_vel_yaw,
                self._format_optional_float(self._trigger_when_x_ge),
            )
        )

    def _elapsed_sec(self) -> float:
        return max(0.0, (int(self.get_clock().now().nanoseconds) - self._started_ns) / 1e9)

    def _on_odom(self, msg: Odometry) -> None:
        self._last_odom_ns = int(self.get_clock().now().nanoseconds)
        self._latest_odom_x = float(msg.pose.pose.position.x)

    def _on_joint_state(self, _: JointState) -> None:
        self._last_joint_state_ns = int(self.get_clock().now().nanoseconds)

    def _is_stack_ready(self) -> bool:
        if not self._wait_for_stack_ready:
            return True

        now_ns = int(self.get_clock().now().nanoseconds)
        freshness_ns = int(self._ready_freshness_sec * 1e9)
        return (
            self._last_odom_ns is not None
            and self._last_joint_state_ns is not None
            and (now_ns - self._last_odom_ns) <= freshness_ns
            and (now_ns - self._last_joint_state_ns) <= freshness_ns
        )

    def _is_trigger_pose_ready(self) -> bool:
        if not math.isfinite(self._trigger_when_x_ge):
            return True
        return (
            self._latest_odom_x is not None
            and self._latest_odom_x >= self._trigger_when_x_ge
        )

    def _approach_enabled(self) -> bool:
        return (
            abs(self._approach_cmd_vel_x) > 1e-6
            or abs(self._approach_cmd_vel_y) > 1e-6
            or abs(self._approach_cmd_vel_yaw) > 1e-6
        )

    def _publish_cmd_vel(self, x: float, y: float, yaw: float) -> None:
        msg = Twist()
        msg.linear.x = float(x)
        msg.linear.y = float(y)
        msg.angular.z = float(yaw)
        self._cmd_vel_pub.publish(msg)

    @staticmethod
    def _format_optional_float(value: Optional[float]) -> str:
        if value is None or not math.isfinite(value):
            return "none"
        return "%.3f" % value

    def _maybe_log_waiting(self) -> None:
        now_ns = int(self.get_clock().now().nanoseconds)
        if now_ns - self._last_wait_log_ns < int(2.0 * 1e9):
            return

        def _age_or_none(stamp_ns: Optional[int]) -> str:
            if stamp_ns is None:
                return "none"
            return "%.2fs" % max(0.0, (now_ns - stamp_ns) / 1e9)

        self.get_logger().info(
            "Waiting for research stack readiness before crossing trigger: "
            "odom_age=%s joint_state_age=%s odom_x=%s trigger_when_x_ge=%s"
            % (
                _age_or_none(self._last_odom_ns),
                _age_or_none(self._last_joint_state_ns),
                self._format_optional_float(self._latest_odom_x),
                self._format_optional_float(self._trigger_when_x_ge),
            )
        )
        self._last_wait_log_ns = now_ns

    def _on_timer(self) -> None:
        if self._fired or not self._enabled:
            return

        if self._elapsed_sec() < self._delay_sec:
            return

        if not self._is_stack_ready():
            self._maybe_log_waiting()
            return

        if not self._is_trigger_pose_ready():
            if self._approach_enabled():
                self._publish_cmd_vel(
                    self._approach_cmd_vel_x,
                    self._approach_cmd_vel_y,
                    self._approach_cmd_vel_yaw,
                )
            self._maybe_log_waiting()
            return

        msg = Bool()
        msg.data = True
        self._publisher.publish(msg)
        if self._stop_cmd_vel_on_trigger:
            self._publish_cmd_vel(0.0, 0.0, 0.0)
        self._fired = True
        self._timer.cancel()
        self.get_logger().info("Published one-shot crossing trigger on %s" % self._topic)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = CrossingTrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
