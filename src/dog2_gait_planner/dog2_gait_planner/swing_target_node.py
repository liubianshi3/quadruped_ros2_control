#!/usr/bin/env python3
"""Raibert foothold + Bezier swing target publisher."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

import numpy as np
import rclpy
from dog2_interfaces.msg import ContactPhase, GaitCommand
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

LEG_NAMES = ["lf", "lh", "rh", "rf"]
# True stance footholds (base_link frame) for the ACTIVE stage-1 stand pose
# femur=+1.05, tibia=-1.40 (body height 0.205 m, matching the MPC
# stance_joint_pose / vertical_support_target_height=0.20), rails locked
# at 0, from Pinocchio FK of the symmetric URDF. These must track whatever
# pose the height loop actually holds: the previous table was the FULL
# stance (tibia -1.10, h=0.268) while the body stood at ~0.19-0.20, so
# every swing touchdown aimed ~6-7 cm below the real ground and stomped,
# scrubbing the body sideways (run32: 1.6 m planar careen vs 0.02 m
# forward projection).
NOMINAL_FOOTS = np.array(
    [
        [-0.134, -0.118, -0.205],
        [0.143, -0.118, -0.205],
        [0.143, 0.118, -0.205],
        [-0.134, 0.118, -0.205],
    ],
    dtype=float,
)


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def cubic_bezier(p0: np.ndarray, pf: np.ndarray, phase: float) -> np.ndarray:
    s = clamp01(phase)
    return p0 + (pf - p0) * (3.0 * s * s - 2.0 * s * s * s)


def cubic_bezier_derivative(p0: np.ndarray, pf: np.ndarray, phase: float) -> np.ndarray:
    s = clamp01(phase)
    return (pf - p0) * (6.0 * s * (1.0 - s))


def swing_bezier(
    p0: np.ndarray,
    pf: np.ndarray,
    phase: float,
    swing_time: float,
    height: float,
) -> tuple[np.ndarray, np.ndarray]:
    safe_time = max(1e-3, swing_time)
    pos = cubic_bezier(p0, pf, phase)
    vel = cubic_bezier_derivative(p0, pf, phase) / safe_time

    if phase < 0.5:
        z0 = np.array([p0[2]], dtype=float)
        zf = np.array([p0[2] + height], dtype=float)
        z_phase = phase * 2.0
        pos[2] = cubic_bezier(z0, zf, z_phase)[0]
        vel[2] = cubic_bezier_derivative(z0, zf, z_phase)[0] * 2.0 / safe_time
    else:
        z0 = np.array([p0[2] + height], dtype=float)
        zf = np.array([pf[2]], dtype=float)
        z_phase = phase * 2.0 - 1.0
        pos[2] = cubic_bezier(z0, zf, z_phase)[0]
        vel[2] = cubic_bezier_derivative(z0, zf, z_phase)[0] * 2.0 / safe_time

    return pos, vel


@dataclass
class LegSwingState:
    in_swing: bool = False
    start_time: float = 0.0
    p0: np.ndarray = None
    pf: np.ndarray = None


class SwingTargetNode(Node):
    def __init__(self) -> None:
        super().__init__("dog2_swing_target")

        self.declare_parameter("contact_phase_topic", "/dog2/gait/contact_phase")
        self.declare_parameter("odom_topic", "/dog2/state_estimation/odom")
        self.declare_parameter("gait_command_topic", "/dog2/gait/command")
        self.declare_parameter("swing_target_topic", "/dog2/gait/swing_foot_target")
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("swing_fraction", 0.5)
        self.declare_parameter("swing_height", 0.08)
        self.declare_parameter("raibert_k", 0.03)
        self.declare_parameter("default_cmd_x", 0.0)
        self.declare_parameter("default_cmd_y", 0.0)

        self._swing_fraction = clamp01(float(self.get_parameter("swing_fraction").value))
        self._swing_height = float(self.get_parameter("swing_height").value)
        self._raibert_k = float(self.get_parameter("raibert_k").value)
        self._cycle_time = 0.8
        self._body_velocity = np.zeros(2, dtype=float)
        self._body_yaw = 0.0
        self._cmd_velocity = np.array(
            [
                float(self.get_parameter("default_cmd_x").value),
                float(self.get_parameter("default_cmd_y").value),
            ],
            dtype=float,
        )
        self._target_pos = NOMINAL_FOOTS.copy()
        self._target_vel = np.zeros((4, 3), dtype=float)
        self._states = [
            LegSwingState(False, 0.0, NOMINAL_FOOTS[i].copy(), NOMINAL_FOOTS[i].copy())
            for i in range(4)
        ]
        self._mask = np.zeros(4, dtype=float)

        self.create_subscription(
            ContactPhase,
            str(self.get_parameter("contact_phase_topic").value),
            self._on_contact_phase,
            20,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_odom,
            20,
        )
        self.create_subscription(
            GaitCommand,
            str(self.get_parameter("gait_command_topic").value),
            self._on_gait_command,
            20,
        )
        self._pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("swing_target_topic").value),
            10,
        )
        rate = max(1.0, float(self.get_parameter("publish_rate").value))
        self.create_timer(1.0 / rate, self._publish)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_odom(self, msg: Odometry) -> None:
        # Odom twist is WORLD frame (gz_pose_to_odom publishes v_world) while
        # footholds live in base_link: rotate the planar velocity by -yaw.
        # Without this, any yaw made the Raibert term push footholds in a
        # body-frame direction that no longer matched the actual motion, and
        # the placement error grew with the very yaw drift it should damp.
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._body_yaw = float(np.arctan2(siny_cosp, cosy_cosp))
        vx_w = msg.twist.twist.linear.x
        vy_w = msg.twist.twist.linear.y
        cy = float(np.cos(self._body_yaw))
        sy = float(np.sin(self._body_yaw))
        self._body_velocity[:] = [
            cy * vx_w + sy * vy_w,
            -sy * vx_w + cy * vy_w,
        ]

    def _on_gait_command(self, msg: GaitCommand) -> None:
        self._cmd_velocity[:] = [msg.linear_x, msg.linear_y]

    def _foothold(self, leg: int) -> np.ndarray:
        stance_time = self._cycle_time * (1.0 - self._swing_fraction)
        foot = NOMINAL_FOOTS[leg].copy()
        foot[:2] += 0.5 * stance_time * self._body_velocity
        foot[:2] += self._raibert_k * (self._body_velocity - self._cmd_velocity)
        return foot

    def _on_contact_phase(self, msg: ContactPhase) -> None:
        self._cycle_time = max(0.1, float(msg.cycle_time))
        now = self._now_sec()
        count = min(len(msg.leg_names), len(msg.phase))
        for i in range(count):
            if msg.leg_names[i] not in LEG_NAMES:
                continue
            leg = LEG_NAMES.index(msg.leg_names[i])
            swing = msg.phase[i] == ContactPhase.SWING
            state = self._states[leg]
            if swing and not state.in_swing:
                state.start_time = now
                state.p0 = self._target_pos[leg].copy()
                state.pf = self._foothold(leg)
            if not swing:
                self._target_pos[leg] = NOMINAL_FOOTS[leg]
                self._target_vel[leg].fill(0.0)
            state.in_swing = swing
            self._mask[leg] = 1.0 if swing else 0.0

    def _publish(self) -> None:
        now = self._now_sec()
        swing_time = max(1e-3, self._cycle_time * self._swing_fraction)
        for leg, state in enumerate(self._states):
            if not state.in_swing:
                continue
            phase = clamp01((now - state.start_time) / swing_time)
            pos, vel = swing_bezier(
                state.p0,
                state.pf,
                phase,
                swing_time,
                self._swing_height,
            )
            self._target_pos[leg] = pos
            self._target_vel[leg] = vel

        msg = Float64MultiArray()
        msg.data = (
            self._mask.tolist()
            + self._target_pos.reshape(-1).tolist()
            + self._target_vel.reshape(-1).tolist()
        )
        self._pub.publish(msg)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = SwingTargetNode()
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
