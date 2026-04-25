#!/usr/bin/env python3
"""Adapt legacy MPC outputs to the unified Dog2 interface topics."""

from __future__ import annotations

from typing import List, Optional

import rclpy
from dog2_interfaces.msg import GRFReference, MPCDebug, MPCHorizon
from geometry_msgs.msg import Point, Pose, PoseArray, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

LEG_NAMES = ["lf", "lh", "rh", "rf"]


class MpcDebugAdapter(Node):
    def __init__(self) -> None:
        super().__init__("dog2_mpc_debug_adapter")

        self.declare_parameter("foot_force_topic", "/dog2/mpc/foot_forces")
        self.declare_parameter("odom_topic", "/dog2/state_estimation/odom")
        self.declare_parameter("grf_topic", "/dog2/mpc/grf_reference")
        self.declare_parameter("debug_topic", "/dog2/mpc/debug")
        self.declare_parameter("horizon_topic", "/dog2/mpc/horizon")
        self.declare_parameter("mode", "stage3_simplified")
        self.declare_parameter("horizon_steps", 10)
        self.declare_parameter("horizon_dt", 0.05)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("publish_zero_force_fallback", True)

        self._mode = str(self.get_parameter("mode").value)
        self._horizon_steps = max(1, int(self.get_parameter("horizon_steps").value))
        self._horizon_dt = float(self.get_parameter("horizon_dt").value)
        self._publish_zero_force_fallback = bool(
            self.get_parameter("publish_zero_force_fallback").value
        )
        self._last_force_msg: Optional[Float64MultiArray] = None
        self._last_odom: Optional[Odometry] = None

        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("foot_force_topic").value),
            self._on_force,
            20,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_odom,
            20,
        )

        self._grf_pub = self.create_publisher(GRFReference, str(self.get_parameter("grf_topic").value), 10)
        self._debug_pub = self.create_publisher(MPCDebug, str(self.get_parameter("debug_topic").value), 10)
        self._horizon_pub = self.create_publisher(MPCHorizon, str(self.get_parameter("horizon_topic").value), 10)

        publish_rate = max(1.0, float(self.get_parameter("publish_rate").value))
        self.create_timer(1.0 / publish_rate, self._publish_debug)

    def _on_force(self, msg: Float64MultiArray) -> None:
        self._last_force_msg = msg

    def _on_odom(self, msg: Odometry) -> None:
        self._last_odom = msg

    def _build_grf_reference(self) -> tuple[Optional[GRFReference], bool]:
        used_fallback = False
        if self._last_force_msg is None or len(self._last_force_msg.data) < 12:
            if self._last_odom is None or not self._publish_zero_force_fallback:
                return None, used_fallback
            used_fallback = True

        msg = GRFReference()
        if self._last_odom is not None:
            msg.header = self._last_odom.header
        msg.leg_names = list(LEG_NAMES)
        msg.forces = []
        for leg_index in range(4):
            base = leg_index * 3
            if used_fallback:
                fx = 0.0
                fy = 0.0
                fz = 0.0
            else:
                fx = float(self._last_force_msg.data[base + 0])
                fy = float(self._last_force_msg.data[base + 1])
                fz = float(self._last_force_msg.data[base + 2])
            msg.forces.append(
                Vector3(
                    x=fx,
                    y=fy,
                    z=fz,
                )
            )
        return msg, used_fallback

    def _build_horizon(self) -> Optional[MPCHorizon]:
        if self._last_odom is None:
            return None

        msg = MPCHorizon()
        msg.header = self._last_odom.header
        msg.base_poses = PoseArray()
        msg.base_poses.header = self._last_odom.header
        msg.com_points = []

        pose = self._last_odom.pose.pose
        twist = self._last_odom.twist.twist
        for step in range(self._horizon_steps):
            dt = step * self._horizon_dt
            sample_pose = Pose()
            sample_pose.position.x = pose.position.x + twist.linear.x * dt
            sample_pose.position.y = pose.position.y + twist.linear.y * dt
            sample_pose.position.z = pose.position.z
            sample_pose.orientation = pose.orientation
            msg.base_poses.poses.append(sample_pose)
            msg.com_points.append(
                Point(
                    x=sample_pose.position.x,
                    y=sample_pose.position.y,
                    z=sample_pose.position.z,
                )
            )
        return msg

    def _publish_debug(self) -> None:
        grf_msg, used_fallback = self._build_grf_reference()
        horizon_msg = self._build_horizon()
        if grf_msg is None or horizon_msg is None:
            return

        debug_msg = MPCDebug()
        debug_msg.header = horizon_msg.header
        debug_msg.mode = self._mode
        debug_msg.solve_time_ms = 0.0
        debug_msg.used_fallback = used_fallback
        debug_msg.base_horizon = horizon_msg.base_poses
        debug_msg.com_horizon = list(horizon_msg.com_points)
        debug_msg.grf_reference = grf_msg

        self._grf_pub.publish(grf_msg)
        self._horizon_pub.publish(horizon_msg)
        self._debug_pub.publish(debug_msg)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = MpcDebugAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
