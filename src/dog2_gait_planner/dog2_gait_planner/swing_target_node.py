#!/usr/bin/env python3
"""Raibert foothold + Bezier swing target publisher."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

import numpy as np
import pinocchio as pin
import rclpy
from dog2_interfaces.msg import ContactPhase, GaitCommand
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

LEG_NAMES = ["lf", "lh", "rh", "rf"]
# Walking footholds (base_link frame) at the stage-1 stand height
# (0.205 m, rails locked at 0). Two truths bound these values:
#
# 1. z must track the height the loop actually holds: the old FULL-stance
#    table (h=0.268) made every touchdown stomp 6-7 cm into the ground
#    (run32: 1.6 m planar careen vs 0.02 m forward projection).
# 2. x is COM-centred, NOT the neutral-pose FK. The neutral stance
#    (femur 1.05 / tibia -1.40) puts the feet at x = -0.134/+0.143,
#    whose diagonal midpoint (x=0.0045) sits 4.2 cm from the COM
#    (x=+0.046): every trot pair then carries a ~3 N*m moment about the
#    support line -- the one axis a two-point contact cannot actuate --
#    and the trunk free-tipped every half cycle (run40/41 forward tilt
#    1-2.4 rad). Centring the feet fore-aft on the COM puts both trot
#    diagonals through the COM and zeroes that moment by geometry.
NOMINAL_FOOTS = np.array(
    [
        [-0.092, -0.118, -0.205],
        [0.184, -0.118, -0.205],
        [0.184, 0.118, -0.205],
        [-0.092, 0.118, -0.205],
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
    """Lift vertically, transfer while clear, then lower vertically."""
    safe_time = max(1e-3, swing_time)
    s = clamp01(phase)
    lift_end = 0.30
    lower_start = 0.70
    apex_z = max(float(p0[2]), float(pf[2])) + height
    pos = p0.copy()
    vel = np.zeros(3, dtype=float)

    if s < lift_end:
        local = s / lift_end
        z0 = np.array([p0[2]], dtype=float)
        zf = np.array([apex_z], dtype=float)
        pos[2] = cubic_bezier(z0, zf, local)[0]
        vel[2] = (
            cubic_bezier_derivative(z0, zf, local)[0]
            / (safe_time * lift_end)
        )
    elif s < lower_start:
        span = lower_start - lift_end
        local = (s - lift_end) / span
        pos[:2] = cubic_bezier(p0[:2], pf[:2], local)
        pos[2] = apex_z
        vel[:2] = (
            cubic_bezier_derivative(p0[:2], pf[:2], local)
            / (safe_time * span)
        )
    else:
        local = (s - lower_start) / (1.0 - lower_start)
        pos[:2] = pf[:2]
        z0 = np.array([apex_z], dtype=float)
        zf = np.array([pf[2]], dtype=float)
        pos[2] = cubic_bezier(z0, zf, local)[0]
        vel[2] = (
            cubic_bezier_derivative(z0, zf, local)[0]
            / (safe_time * (1.0 - lower_start))
        )

    return pos, vel


def foothold_velocity_offset(
    body_velocity: np.ndarray,
    command_velocity: np.ndarray,
    *,
    gait: str,
    raibert_k: float,
    crawl_lead_sec: float,
    maximum: float,
) -> np.ndarray:
    """Return bounded feedback plus crawl stance-time velocity lead."""
    offset = raibert_k * (body_velocity - command_velocity)
    if gait == "crawl":
        # A crawl foot remains planted while the other three legs take their
        # turns. Put it ahead of the static nominal at touchdown so that the
        # support polygon still covers the COM near the end of that interval.
        offset += crawl_lead_sec * command_velocity
    norm = float(np.linalg.norm(offset))
    if norm > maximum > 0.0:
        offset *= maximum / norm
    return offset


def touchdown_height(liftoff_height: float, overshoot: float) -> float:
    """Search just below the measured ground height at liftoff."""
    return float(liftoff_height) - max(0.0, float(overshoot))


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
        self.declare_parameter("crawl_velocity_lead_sec", 1.2)
        # Cap on the Raibert foothold shift from nominal: body-velocity
        # transients (tip/careen recoveries reach ~1 m/s) otherwise send
        # footholds 20+ cm out, beyond the leg workspace.
        self.declare_parameter("foothold_offset_max", 0.06)
        # Ground-search overshoot below the measured liftoff ground height.
        # A fixed nominal z drove short legs several centimetres beyond their
        # reachable workspace when the trunk rode below its nominal height.
        self.declare_parameter("touchdown_overshoot", 0.02)
        self.declare_parameter("default_cmd_x", 0.0)
        self.declare_parameter("default_cmd_y", 0.0)
        self.declare_parameter("robot_description", "")
        self.declare_parameter("joint_state_topic", "/joint_states")

        self._swing_fraction = clamp01(float(self.get_parameter("swing_fraction").value))
        self._swing_height = float(self.get_parameter("swing_height").value)
        self._raibert_k = float(self.get_parameter("raibert_k").value)
        self._crawl_velocity_lead_sec = max(
            0.0, float(self.get_parameter("crawl_velocity_lead_sec").value)
        )
        self._foothold_offset_max = max(
            0.0, float(self.get_parameter("foothold_offset_max").value)
        )
        self._touchdown_overshoot = max(
            0.0, float(self.get_parameter("touchdown_overshoot").value)
        )
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
        self._cmd_yaw_rate = 0.0
        self._gait = "trot"
        self._target_pos = NOMINAL_FOOTS.copy()
        self._target_vel = np.zeros((4, 3), dtype=float)
        self._states = [
            LegSwingState(False, 0.0, NOMINAL_FOOTS[i].copy(), NOMINAL_FOOTS[i].copy())
            for i in range(4)
        ]
        self._mask = np.zeros(4, dtype=float)
        self._actual_foot_pos = NOMINAL_FOOTS.copy()
        self._actual_foot_valid = np.zeros(4, dtype=bool)
        self._pin_model = None
        self._pin_data = None
        self._pin_q = None
        self._joint_q_index: dict[str, int] = {}
        self._foot_frame_id: list[int] = []
        self._initialize_kinematics(
            str(self.get_parameter("robot_description").value)
        )

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
        if self._pin_model is not None:
            self.create_subscription(
                JointState,
                str(self.get_parameter("joint_state_topic").value),
                self._on_joint_state,
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

    def _initialize_kinematics(self, robot_description: str) -> None:
        if not robot_description:
            self.get_logger().warning(
                "robot_description is empty; swing starts use nominal feet"
            )
            return
        self._pin_model = pin.buildModelFromXML(robot_description)
        self._pin_data = self._pin_model.createData()
        self._pin_q = np.zeros(self._pin_model.nq)
        for leg in LEG_NAMES:
            for suffix in (
                "rail_joint",
                "coxa_joint",
                "femur_joint",
                "tibia_joint",
            ):
                name = f"{leg}_{suffix}"
                joint_id = self._pin_model.getJointId(name)
                if joint_id == 0:
                    raise RuntimeError(f"swing model is missing {name}")
                self._joint_q_index[name] = int(
                    self._pin_model.idx_qs[joint_id]
                )
            frame_id = self._pin_model.getFrameId(f"{leg}_foot_link")
            if frame_id >= self._pin_model.nframes:
                raise RuntimeError(f"swing model is missing {leg}_foot_link")
            self._foot_frame_id.append(int(frame_id))

    def _on_joint_state(self, msg: JointState) -> None:
        assert self._pin_model is not None
        assert self._pin_data is not None
        assert self._pin_q is not None
        updated = False
        for name, position in zip(msg.name, msg.position):
            q_index = self._joint_q_index.get(name)
            if q_index is None or not np.isfinite(position):
                continue
            self._pin_q[q_index] = float(position)
            updated = True
        if not updated:
            return
        pin.forwardKinematics(self._pin_model, self._pin_data, self._pin_q)
        pin.updateFramePlacements(self._pin_model, self._pin_data)
        for leg, frame_id in enumerate(self._foot_frame_id):
            position = np.asarray(
                self._pin_data.oMf[frame_id].translation, dtype=float
            ).reshape(3)
            if np.all(np.isfinite(position)):
                self._actual_foot_pos[leg] = position
                self._actual_foot_valid[leg] = True

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
        v_body = np.array(
            [cy * vx_w + sy * vy_w, -sy * vx_w + cy * vy_w], dtype=float
        )
        # ~0.3 s low-pass: strip the trot rock (base-frame velocity swings
        # +-0.12 m/s at gait frequency) before it feeds the Raibert term.
        self._body_velocity += 0.15 * (v_body - self._body_velocity)

    def _on_gait_command(self, msg: GaitCommand) -> None:
        self._gait = msg.gait or self._gait
        self._cmd_velocity[:] = [msg.linear_x, msg.linear_y]
        self._cmd_yaw_rate = float(msg.angular_z)

    def _foothold(self, leg: int) -> np.ndarray:
        foot = NOMINAL_FOOTS[leg].copy()
        # Trot keeps the COM-centred nominal so both diagonal support lines
        # pass through the COM. Crawl instead needs a velocity lead because
        # each foot remains planted through the other three single-leg slots.
        # The common workspace cap bounds both that lead and Raibert feedback.
        offset = foothold_velocity_offset(
            self._body_velocity,
            self._cmd_velocity,
            gait=self._gait,
            raibert_k=self._raibert_k,
            crawl_lead_sec=self._crawl_velocity_lead_sec,
            maximum=self._foothold_offset_max,
        )
        foot[:2] += offset
        # Yaw lead: rotate the foothold by half the yaw the body will cover
        # during the stance. Without it a commanded turn lands the feet at
        # the unrotated nominal and the body twists on planted feet --
        # lateral scrub, tip, height dips (run65 turn z=0.083).
        stance_time = self._cycle_time * (1.0 - self._swing_fraction)
        dtheta = 0.5 * stance_time * self._cmd_yaw_rate
        if abs(dtheta) > 1e-6:
            c, s = float(np.cos(dtheta)), float(np.sin(dtheta))
            x, y = foot[0], foot[1]
            foot[0] = c * x - s * y
            foot[1] = s * x + c * y
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
                state.p0 = (
                    self._actual_foot_pos[leg].copy()
                    if self._actual_foot_valid[leg]
                    else self._target_pos[leg].copy()
                )
                state.pf = self._foothold(leg)
                state.pf[2] = touchdown_height(
                    state.p0[2], self._touchdown_overshoot
                )
                self._target_pos[leg] = state.p0.copy()
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
