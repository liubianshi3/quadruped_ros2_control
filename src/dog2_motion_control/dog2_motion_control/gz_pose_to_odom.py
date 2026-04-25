#!/usr/bin/env python3
"""Bridge Gazebo dynamic pose stream to nav_msgs/Odometry for MPC fallback.

This node is intended as a robust fallback when `/model/.../odometry` exists
as a topic but does not carry data in a specific Gazebo setup.

Pipeline:
1. `/world/.../dynamic_pose/info` is bridged to ROS `tf2_msgs/TFMessage`.
2. This node extracts the configured dog2 base pose from TF-like transforms.
3. Linear / angular velocity are estimated from finite differences.
4. Publish synthesized `nav_msgs/Odometry` on `/odom`.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, sqrt
from typing import Optional, Sequence

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from tf2_msgs.msg import TFMessage


def _normalize_quat_xyzw(q: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / n


def _quat_conjugate_xyzw(q: np.ndarray) -> np.ndarray:
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=float)


def _quat_multiply_xyzw(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ],
        dtype=float,
    )


def _quat_delta_to_omega_world(prev_q: np.ndarray, curr_q: np.ndarray, dt: float) -> np.ndarray:
    if dt <= 1e-6:
        return np.zeros(3, dtype=float)

    dq = _quat_multiply_xyzw(curr_q, _quat_conjugate_xyzw(prev_q))
    dq = _normalize_quat_xyzw(dq)

    # Shortest-arc convention.
    if dq[3] < 0.0:
        dq = -dq

    vec = dq[:3]
    vec_n = float(np.linalg.norm(vec))
    if vec_n < 1e-12:
        return np.zeros(3, dtype=float)

    angle = 2.0 * atan2(vec_n, float(dq[3]))
    axis = vec / vec_n
    return axis * (angle / dt)


def _is_identity_like_transform(tfm: TransformStamped) -> bool:
    tr = tfm.transform.translation
    rot = tfm.transform.rotation
    translation_norm = sqrt(float(tr.x) ** 2 + float(tr.y) ** 2 + float(tr.z) ** 2)
    quat = _normalize_quat_xyzw(
        np.array([float(rot.x), float(rot.y), float(rot.z), float(rot.w)], dtype=float)
    )
    return translation_norm < 1e-6 and np.linalg.norm(quat - np.array([0.0, 0.0, 0.0, 1.0])) < 1e-6


@dataclass
class _PoseSample:
    t_sec: float
    p_world: np.ndarray  # shape (3,)
    q_xyzw: np.ndarray  # shape (4,)


class GzPoseToOdom(Node):
    """Convert bridged Gazebo dynamic pose TF stream into Odometry."""

    def __init__(self) -> None:
        super().__init__("gz_pose_to_odom")

        self.declare_parameter("pose_topic", "/dog2/dynamic_pose_tf")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("external_odom_topic", "")
        self.declare_parameter("model_name", "dog2")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_only_when_no_external_odom", False)
        self.declare_parameter("external_odom_timeout_sec", 0.35)
        self.declare_parameter("min_dt_sec", 0.002)
        self.declare_parameter("max_dt_sec", 0.2)
        self.declare_parameter("max_linear_speed", 8.0)
        self.declare_parameter("max_angular_speed", 20.0)
        self.declare_parameter("twist_lpf_tau", 0.03)

        self._pose_topic = str(self.get_parameter("pose_topic").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._external_odom_topic = str(self.get_parameter("external_odom_topic").value)
        self._model_name = str(self.get_parameter("model_name").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._base_frame_lower = self._base_frame.lower()
        self._publish_only_when_no_external_odom = bool(
            self.get_parameter("publish_only_when_no_external_odom").value
        )
        self._external_odom_timeout_sec = float(
            max(float(self.get_parameter("external_odom_timeout_sec").value), 1e-3)
        )
        self._min_dt_sec = float(max(float(self.get_parameter("min_dt_sec").value), 1e-6))
        self._max_dt_sec = float(max(float(self.get_parameter("max_dt_sec").value), self._min_dt_sec))
        self._max_linear_speed = float(max(float(self.get_parameter("max_linear_speed").value), 0.0))
        self._max_angular_speed = float(max(float(self.get_parameter("max_angular_speed").value), 0.0))
        self._twist_lpf_tau = float(max(float(self.get_parameter("twist_lpf_tau").value), 1e-4))

        qos_pose = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        qos_odom = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._pose_sub = self.create_subscription(
            TFMessage,
            self._pose_topic,
            self._on_pose_tf,
            qos_pose,
        )
        self._external_odom_sub = None
        if self._external_odom_topic and self._external_odom_topic != self._odom_topic:
            self._external_odom_sub = self.create_subscription(
                Odometry,
                self._external_odom_topic,
                self._on_external_odom,
                qos_odom,
            )
        self._odom_pub = self.create_publisher(Odometry, self._odom_topic, qos_odom)
        self._prev_sample: Optional[_PoseSample] = None
        self._last_external_odom_sec: Optional[float] = None
        self._v_world_filt = np.zeros(3, dtype=float)
        self._w_world_filt = np.zeros(3, dtype=float)
        self._last_debug_sec = 0.0

        self.get_logger().info(
            f"gz_pose_to_odom ready: pose_topic='{self._pose_topic}', odom_topic='{self._odom_topic}', "
            f"external_odom_topic='{self._external_odom_topic}', model='{self._model_name}', "
            f"base_frame='{self._base_frame}'"
        )

    @staticmethod
    def _ends_with_frame(child: str, frame: str) -> bool:
        return child.endswith(f"/{frame}") or child.endswith(f"::{frame}")

    def _transform_priority(self, child: str) -> int:
        child_l = child.lower()
        model_l = self._model_name.lower()
        base_l = self._base_frame_lower

        if model_l in child_l and base_l in child_l:
            return 0
        if child_l == base_l:
            return 1
        if self._ends_with_frame(child_l, base_l):
            return 2
        if child_l == model_l:
            return 3
        if model_l in child_l:
            return 4
        return 100

    def _select_transform(self, transforms: Sequence[TransformStamped]) -> Optional[TransformStamped]:
        if not transforms:
            return None

        valid = [
            tfm
            for tfm in transforms
            if self._transform_priority(tfm.child_frame_id) < 100
        ]
        if not valid:
            return None

        ranked = sorted(valid, key=lambda tfm: self._transform_priority(tfm.child_frame_id))
        best = ranked[0]

        model_l = self._model_name.lower()
        base_l = self._base_frame_lower
        exact_model = next(
            (tfm for tfm in valid if tfm.child_frame_id.lower() == model_l),
            None,
        )
        exact_base = next(
            (tfm for tfm in valid if tfm.child_frame_id.lower() == base_l),
            None,
        )

        # Gazebo dynamic_pose sometimes exposes a world-space model pose plus a
        # zero local base_link pose. Prefer the model pose in that case.
        if (
            exact_model is not None
            and exact_base is not None
            and _is_identity_like_transform(exact_base)
            and not _is_identity_like_transform(exact_model)
        ):
            return exact_model

        return best

    @staticmethod
    def _describe_transforms(transforms: Sequence[TransformStamped], limit: int = 8) -> str:
        if not transforms:
            return "[]"

        parts = []
        for tfm in transforms[:limit]:
            parts.append(
                f"{tfm.header.frame_id}->{tfm.child_frame_id}"
            )
        if len(transforms) > limit:
            parts.append(f"...(+{len(transforms) - limit} more)")
        return "[" + ", ".join(parts) + "]"

    def _on_pose_tf(self, msg: TFMessage) -> None:
        if not self.context.ok():
            return

        if not msg.transforms:
            return

        tf_sel = self._select_transform(msg.transforms)
        if tf_sel is None:
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            if now_sec - self._last_debug_sec > 2.0:
                self.get_logger().warn(
                    "dynamic_pose TF received but no transform matched model/base hints. "
                    f"model='{self._model_name}' base='{self._base_frame}' "
                    f"candidates={self._describe_transforms(msg.transforms)}"
                )
                self._last_debug_sec = now_sec
            return

        stamp = tf_sel.header.stamp
        t_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if t_sec <= 0.0:
            t_sec = self.get_clock().now().nanoseconds * 1e-9

        tr = tf_sel.transform.translation
        rot = tf_sel.transform.rotation
        p_world = np.array([float(tr.x), float(tr.y), float(tr.z)], dtype=float)
        q_xyzw = _normalize_quat_xyzw(
            np.array([float(rot.x), float(rot.y), float(rot.z), float(rot.w)], dtype=float)
        )
        sample = _PoseSample(t_sec=t_sec, p_world=p_world, q_xyzw=q_xyzw)

        v_world = np.zeros(3, dtype=float)
        w_world = np.zeros(3, dtype=float)
        if self._prev_sample is not None:
            dt = float(sample.t_sec - self._prev_sample.t_sec)
            if self._min_dt_sec <= dt <= self._max_dt_sec:
                v_world = (sample.p_world - self._prev_sample.p_world) / dt
                w_world = _quat_delta_to_omega_world(self._prev_sample.q_xyzw, sample.q_xyzw, dt)

                # Guard finite-difference spikes before exporting odom twist.
                v_world = np.clip(v_world, -self._max_linear_speed, self._max_linear_speed)
                w_world = np.clip(w_world, -self._max_angular_speed, self._max_angular_speed)

                # Lightweight LPF to keep fallback odom numerically stable for MPC.
                alpha = float(np.clip(dt / (self._twist_lpf_tau + dt), 0.0, 1.0))
                self._v_world_filt += alpha * (v_world - self._v_world_filt)
                self._w_world_filt += alpha * (w_world - self._w_world_filt)
            else:
                self._v_world_filt *= 0.0
                self._w_world_filt *= 0.0
        v_world = self._v_world_filt
        w_world = self._w_world_filt

        self._prev_sample = sample

        # Prefer forwarded Gazebo odom when it is available and fresh.
        if self._has_fresh_external_odom():
            return

        # Backward-compatible safeguard for older launch files that still wire
        # another publisher directly onto the same odom topic.
        if self._publish_only_when_no_external_odom and not self._external_odom_topic:
            if self.count_publishers(self._odom_topic) > 1:
                return

        odom = Odometry()
        if int(stamp.sec) == 0 and int(stamp.nanosec) == 0:
            odom.header.stamp = self.get_clock().now().to_msg()
        else:
            odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame

        odom.pose.pose.position.x = float(sample.p_world[0])
        odom.pose.pose.position.y = float(sample.p_world[1])
        odom.pose.pose.position.z = float(sample.p_world[2])
        odom.pose.pose.orientation.x = float(sample.q_xyzw[0])
        odom.pose.pose.orientation.y = float(sample.q_xyzw[1])
        odom.pose.pose.orientation.z = float(sample.q_xyzw[2])
        odom.pose.pose.orientation.w = float(sample.q_xyzw[3])

        odom.twist.twist.linear.x = float(v_world[0])
        odom.twist.twist.linear.y = float(v_world[1])
        odom.twist.twist.linear.z = float(v_world[2])
        odom.twist.twist.angular.x = float(w_world[0])
        odom.twist.twist.angular.y = float(w_world[1])
        odom.twist.twist.angular.z = float(w_world[2])

        self._publish_odom(odom)

    def _on_external_odom(self, msg: Odometry) -> None:
        self._last_external_odom_sec = self.get_clock().now().nanoseconds * 1e-9
        self._publish_odom(msg)

    def _has_fresh_external_odom(self) -> bool:
        if self._last_external_odom_sec is None:
            return False
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        return (now_sec - self._last_external_odom_sec) <= self._external_odom_timeout_sec

    def _publish_odom(self, msg: Odometry) -> None:
        try:
            self._odom_pub.publish(msg)
        except Exception:
            # During shutdown, context can become invalid between callback and publish.
            return


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GzPoseToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.context.ok():
                node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
