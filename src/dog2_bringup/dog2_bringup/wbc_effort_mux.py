from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String


def _blend_effort(hold: np.ndarray, target: np.ndarray, alpha: float) -> np.ndarray:
    alpha = float(np.clip(alpha, 0.0, 1.0))
    return hold + alpha * (target - hold)


def _quat_to_rot(w: float, x: float, y: float, z: float) -> np.ndarray:
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-9:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


class QuasiStaticStandupFF:
    """按测量位形实时计算起立静力前馈。

    tau_ff(q) = g(q) - sum_i J_i(q)^T R_wb^T [0, 0, fz_i(q)]

    fz_i(q): 与 MPC applyVerticalSupport 同款的最小偏差垂直力分配 --
    总和 = 全机重量, 对实时 COM 的世界系水平力矩为零, 力臂取
    R_wb * (foot - com)。

    常量前馈只在单一姿态精确: 站姿版在中间深蹲欠补偿 (重力矩远大于
    前馈), 蹲姿版又变成折叠偏置, run19-26 两种都停在 h~0.09、离设计
    蹲姿差 0.3 rad。逐周期重算后, 任意中间姿态都精确平衡, PD 只负责
    把姿态推向目标。
    """

    _LEG_PREFIXES = ("lf", "lh", "rh", "rf")
    _JOINT_SUFFIXES = ("rail_joint", "coxa_joint", "femur_joint", "tibia_joint")

    def __init__(self, robot_description_xml: str) -> None:
        import pinocchio as pin  # 可选依赖，仅在启用在线前馈时导入

        self._pin = pin
        self._model = pin.buildModelFromXML(robot_description_xml)
        self._data = self._model.createData()

        self._q_index = np.zeros((4, 4), dtype=int)
        self._v_index = np.zeros((4, 4), dtype=int)
        for leg, prefix in enumerate(self._LEG_PREFIXES):
            for j, suffix in enumerate(self._JOINT_SUFFIXES):
                name = f"{prefix}_{suffix}"
                if not self._model.existJointName(name):
                    raise ValueError(f"URDF missing joint {name}")
                jid = self._model.getJointId(name)
                self._q_index[leg, j] = self._model.idx_qs[jid]
                self._v_index[leg, j] = self._model.idx_vs[jid]

        self._foot_fids = []
        for prefix in self._LEG_PREFIXES:
            fname = f"{prefix}_foot_link"
            if not self._model.existFrame(fname):
                raise ValueError(f"URDF missing frame {fname}")
            self._foot_fids.append(self._model.getFrameId(fname))

        # pin.centerOfMass / computeTotalMass 跳过 universe 惯量, 而固定
        # 基座模型的躯干 (半机重) 恰好焊在 universe 上, 必须手动并入
        # (与 Dog2Model::centerOfMass 同一个修复)。
        inert_u = self._model.inertias[0]
        self._universe_mass = float(inert_u.mass)
        self._universe_lever = np.asarray(inert_u.lever, dtype=float).copy()
        self._legs_mass = float(pin.computeTotalMass(self._model))
        self._total_mass = self._universe_mass + self._legs_mass

    @property
    def total_mass(self) -> float:
        return self._total_mass

    def compute(
        self,
        joint_pos12: np.ndarray,
        rail_pos4: np.ndarray,
        r_wb: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """返回 (tau12 [lf,lh,rh,rf]x[coxa,femur,tibia], fz4 世界系垂直力)。"""
        pin = self._pin
        model, data = self._model, self._data

        q = np.zeros(model.nq)
        for leg in range(4):
            q[self._q_index[leg, 0]] = float(rail_pos4[leg])
            q[self._q_index[leg, 1]] = float(joint_pos12[leg * 3 + 0])
            q[self._q_index[leg, 2]] = float(joint_pos12[leg * 3 + 1])
            q[self._q_index[leg, 3]] = float(joint_pos12[leg * 3 + 2])

        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        com_legs = np.asarray(pin.centerOfMass(model, data, q), dtype=float)
        com = (
            self._legs_mass * com_legs + self._universe_mass * self._universe_lever
        ) / self._total_mass
        feet = [
            np.asarray(data.oMf[fid].translation, dtype=float).copy()
            for fid in self._foot_fids
        ]

        # 世界系水平力臂 + 最小偏差静力分配 (总量守恒, 对 COM 力矩为零)
        total_force = self._total_mass * 9.81
        base = total_force / 4.0
        levers = [r_wb @ (p - com) for p in feet]
        sxx, sxy, syy, sx, sy = 1e-4, 0.0, 0.0, 0.0, 0.0
        for d in levers:
            sxx += d[0] * d[0]
            sxy += d[0] * d[1]
            syy += d[1] * d[1]
            sx += d[0]
            sy += d[1]
        syy += 1e-4
        det = sxx * syy - sxy * sxy
        l1 = l2 = 0.0
        if abs(det) > 1e-9:
            b1 = -base * sx
            b2 = -base * sy
            l1 = (b1 * syy - b2 * sxy) / det
            l2 = (b2 * sxx - b1 * sxy) / det
        fz = np.array(
            [max(0.0, base + l1 * d[0] + l2 * d[1]) for d in levers], dtype=float
        )
        fz_sum = float(fz.sum())
        if fz_sum > 1e-6:
            fz *= total_force / fz_sum

        # 躯干倾斜时重力在 base 系中的真实方向
        model.gravity.linear = r_wb.T @ np.array([0.0, 0.0, -9.81])
        g = pin.rnea(
            model, data, q, np.zeros(model.nv), np.zeros(model.nv)
        )

        # 静平衡: tau = g(q) - J^T f_ext, f_ext 为地面作用于足端的支撑力
        tau_full = np.asarray(g, dtype=float).copy()
        for leg in range(4):
            jac = pin.computeFrameJacobian(
                model, data, q, self._foot_fids[leg], pin.LOCAL_WORLD_ALIGNED
            )
            f_base = r_wb.T @ np.array([0.0, 0.0, fz[leg]])
            tau_full -= jac[:3, :].T @ f_base

        tau12 = np.zeros(12)
        for leg in range(4):
            for j in range(3):
                tau12[leg * 3 + j] = tau_full[self._v_index[leg, j + 1]]
        return tau12, fz


class WBCEffortMux(Node):
    _LEG_NAMES = ("lf", "lh", "rh", "rf")

    def __init__(self) -> None:
        super().__init__("wbc_effort_mux")

        self.declare_parameter("joint_effort_topic", "/dog2/wbc/joint_effort_command")
        self.declare_parameter("rail_effort_topic", "/dog2/wbc/rail_effort_command")
        self.declare_parameter("output_topic", "/effort_controller/commands")
        self.declare_parameter("publish_rate_hz", 200.0)
        self.declare_parameter("command_timeout_sec", 0.25)
        self.declare_parameter("debug_enabled", True)
        self.declare_parameter("debug_log_period_sec", 1.0)
        self.declare_parameter("publish_safe_zero_on_stale", False)
        self.declare_parameter("startup_hold_enabled", True)
        self.declare_parameter("startup_hold_sec", 1.0)
        self.declare_parameter("startup_ramp_sec", 1.0)
        # Gravity/support feed-forward at the standing pose (J^T*[0,0,mg/4]+g).
        self.declare_parameter("startup_hold_joint_effort", [0.0, -7.029, -9.362])
        # Joint-space PD stand-up during the startup phase. The robot is limp
        # (0 N effort joints) while the controller chain comes up and always
        # lands in a partially collapsed heap; a static torque vector cannot
        # recover from an arbitrary heap, a PD toward the standing pose can.
        # The PD target is ramped from the measured pose to the stance over
        # startup_standup_pose_ramp_sec so the rise is quasi-static; snapping
        # straight to the stance pose tips the robot into a pitched rest.
        self.declare_parameter("startup_standup_pd_enabled", True)
        self.declare_parameter("startup_standup_pose", [0.0, 1.05, -1.10])
        self.declare_parameter("startup_standup_pose_ramp_sec", 3.0)
        self.declare_parameter("startup_standup_kp", [18.0, 30.0, 30.0])
        self.declare_parameter("startup_standup_kd", [0.6, 1.2, 1.2])
        self.declare_parameter("startup_standup_max_torque", 28.0)
        # The hold timer must start when the effort controller can actually
        # consume commands. A bare get_subscription_count() is wrong for
        # that: diagnostic listeners (e.g. smoke_check) also subscribe to
        # the output topic, which used to start the timer seconds before
        # the controller activated and burned the pose ramp while the
        # joints were still limp (run15). Match the subscriber node name.
        self.declare_parameter("output_controller_node", "effort_controller")
        # Handoff gate: blending into the WBC while the trunk is still
        # pitched or the joints are still moving hands a bad state to a
        # stack with no attitude authority in HOVER (run15 flipped exactly
        # at the fixed-timer handoff). Keep holding the stand-up PD until
        # the robot is quiet, with a hard cap so a broken run still fails
        # visibly instead of deadlocking.
        self.declare_parameter("startup_handoff_require_settle", True)
        self.declare_parameter(
            "startup_handoff_odom_topic", "/dog2/state_estimation/odom"
        )
        self.declare_parameter("startup_handoff_tilt_max_rad", 0.15)
        self.declare_parameter("startup_handoff_joint_speed_max_rad_s", 0.6)
        self.declare_parameter("startup_handoff_settle_duration_sec", 1.0)
        self.declare_parameter("startup_handoff_max_extra_sec", 30.0)
        self.declare_parameter("startup_handoff_no_odom_grace_sec", 5.0)
        # Trunk leveling for the stand-up PD. A pure joint-space PD is blind
        # to trunk attitude: with the 42/58 front/hind load split the rise
        # reliably converges to a pitched rest (run16: hind tibia sagged
        # 0.3 rad more than front, trunk pitched 0.6 rad, zero velocity).
        # pitch > 0 = tail-side (+X, hind feet) down, so extend the hind
        # legs and fold the front legs by a differential target offset.
        self.declare_parameter("startup_standup_level_enabled", True)
        self.declare_parameter("startup_standup_level_kp", 0.5)
        self.declare_parameter("startup_standup_level_max_delta_rad", 0.30)
        # Scale the static feed-forward with the pose ramp progress. The
        # feed-forward is exact only at the stance pose; applying it in
        # full while the robot still lies collapsed injects an asymmetric
        # folding torque (hind FF is larger than front), which seeded the
        # pitch runaway during the rise (run16/17). At s=0 the ground
        # carries the body and no feed-forward is needed; both endpoints
        # are exact under scaling.
        self.declare_parameter("startup_hold_effort_ramp_enabled", True)
        # Online quasi-static feed-forward: recompute J^T f + g at the
        # MEASURED pose every cycle instead of a constant vector baked for
        # one design pose. Constant FF is the reason runs 19-26 stalled at
        # h~0.09: at intermediate crouch poses the true gravity moments far
        # exceed the stance-pose FF and the PD alone cannot close the gap.
        # Requires robot_description; falls back to the constant vector.
        self.declare_parameter("startup_online_ff_enabled", True)
        self.declare_parameter("robot_description", "")
        self.declare_parameter("startup_online_ff_period_sec", 0.02)
        self.declare_parameter("freeze_rail_effort", False)
        self.declare_parameter("freeze_rail_effort_in_hover", False)
        self.declare_parameter("freeze_rail_effort_in_walking", False)
        self.declare_parameter("freeze_rail_effort_in_crossing_staging", False)
        self.declare_parameter("freeze_rail_effort_on_flat", False)
        self.declare_parameter("crossing_state_topic", "/dog2/mpc/crossing_state")
        # False = rails are locked by a dedicated position controller, the
        # effort_controller only drives the 12 rotational joints and the WBC
        # rail effort stream is ignored (12-channel output layout).
        self.declare_parameter("include_rail_in_output", True)

        self._joint_effort_topic = str(self.get_parameter("joint_effort_topic").value)
        self._rail_effort_topic = str(self.get_parameter("rail_effort_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._command_timeout_sec = float(self.get_parameter("command_timeout_sec").value)
        self._debug_enabled = bool(self.get_parameter("debug_enabled").value)
        self._debug_log_period_sec = max(0.1, float(self.get_parameter("debug_log_period_sec").value))
        self._freeze_rail_effort = bool(self.get_parameter("freeze_rail_effort").value)
        self._freeze_rail_effort_in_hover = bool(
            self.get_parameter("freeze_rail_effort_in_hover").value
        )
        self._freeze_rail_effort_in_walking = bool(
            self.get_parameter("freeze_rail_effort_in_walking").value
        )
        self._freeze_rail_effort_in_crossing_staging = bool(
            self.get_parameter("freeze_rail_effort_in_crossing_staging").value
        )
        self._freeze_rail_effort_on_flat = bool(
            self.get_parameter("freeze_rail_effort_on_flat").value
        )
        self._crossing_state_topic = str(
            self.get_parameter("crossing_state_topic").value
        )
        self._include_rail_in_output = bool(
            self.get_parameter("include_rail_in_output").value
        )
        self._publish_safe_zero_on_stale = bool(
            self.get_parameter("publish_safe_zero_on_stale").value
        )
        self._startup_hold_enabled = bool(
            self.get_parameter("startup_hold_enabled").value
        )
        self._startup_hold_sec = max(
            0.0, float(self.get_parameter("startup_hold_sec").value)
        )
        self._startup_ramp_sec = max(
            0.0, float(self.get_parameter("startup_ramp_sec").value)
        )
        hold_joint_effort = [
            float(v) for v in self.get_parameter("startup_hold_joint_effort").value
        ]
        if len(hold_joint_effort) == 3:
            # One [coxa, femur, tibia] triple for all four legs.
            self._startup_hold_joint_effort = np.tile(
                np.asarray(hold_joint_effort, dtype=float), 4
            )
        elif len(hold_joint_effort) == 12:
            # Per-leg triples in mux leg order [lf, lh, rh, rf]: the static
            # load split is front/hind asymmetric (COM sits closer to the
            # hind feet), so uniform feed-forward sags the hind legs.
            self._startup_hold_joint_effort = np.asarray(
                hold_joint_effort, dtype=float
            )
        else:
            raise ValueError(
                "startup_hold_joint_effort must have 3 or 12 entries"
            )
        self._startup_hold_effort = np.asarray(
            self._compose_output(
                self._startup_hold_joint_effort.copy(),
                np.zeros(4, dtype=float),
            ),
            dtype=float,
        )
        self._standup_pd_enabled = bool(
            self.get_parameter("startup_standup_pd_enabled").value
        )
        standup_pose = [float(v) for v in self.get_parameter("startup_standup_pose").value]
        standup_kp = [float(v) for v in self.get_parameter("startup_standup_kp").value]
        standup_kd = [float(v) for v in self.get_parameter("startup_standup_kd").value]
        if len(standup_pose) != 3 or len(standup_kp) != 3 or len(standup_kd) != 3:
            raise ValueError("startup_standup_* must be [coxa, femur, tibia]")
        self._standup_pose = np.tile(np.asarray(standup_pose, dtype=float), 4)
        self._standup_kp = np.tile(np.asarray(standup_kp, dtype=float), 4)
        self._standup_kd = np.tile(np.asarray(standup_kd, dtype=float), 4)
        self._standup_max_torque = float(
            self.get_parameter("startup_standup_max_torque").value
        )
        self._standup_pose_ramp_sec = max(
            0.1, float(self.get_parameter("startup_standup_pose_ramp_sec").value)
        )
        self._output_controller_node = str(
            self.get_parameter("output_controller_node").value
        )
        self._handoff_require_settle = bool(
            self.get_parameter("startup_handoff_require_settle").value
        )
        handoff_odom_topic = str(
            self.get_parameter("startup_handoff_odom_topic").value
        )
        self._handoff_tilt_max_rad = float(
            self.get_parameter("startup_handoff_tilt_max_rad").value
        )
        self._handoff_joint_speed_max = float(
            self.get_parameter("startup_handoff_joint_speed_max_rad_s").value
        )
        self._handoff_settle_duration_sec = float(
            self.get_parameter("startup_handoff_settle_duration_sec").value
        )
        self._handoff_max_extra_sec = float(
            self.get_parameter("startup_handoff_max_extra_sec").value
        )
        self._handoff_no_odom_grace_sec = float(
            self.get_parameter("startup_handoff_no_odom_grace_sec").value
        )
        self._standup_level_enabled = bool(
            self.get_parameter("startup_standup_level_enabled").value
        )
        self._standup_level_kp = float(
            self.get_parameter("startup_standup_level_kp").value
        )
        self._standup_level_max_delta = float(
            self.get_parameter("startup_standup_level_max_delta_rad").value
        )
        self._hold_effort_ramp_enabled = bool(
            self.get_parameter("startup_hold_effort_ramp_enabled").value
        )
        self._standup_initial_pose: Optional[np.ndarray] = None
        self._joint_pos = np.zeros(12, dtype=float)
        self._joint_vel = np.zeros(12, dtype=float)
        self._joint_state_valid = False
        self._joint_index = {}
        for leg_idx, leg in enumerate(self._LEG_NAMES):
            for joint_idx, joint in enumerate(("coxa", "femur", "tibia")):
                self._joint_index[f"{leg}_{joint}_joint"] = leg_idx * 3 + joint_idx
        self._rail_index = {
            f"{leg}_rail_joint": leg_idx
            for leg_idx, leg in enumerate(self._LEG_NAMES)
        }
        self._rail_pos = np.zeros(4, dtype=float)

        # 在线准静态前馈（可选，失败自动回退常量前馈）
        self._online_ff: Optional[QuasiStaticStandupFF] = None
        self._online_ff_period_sec = max(
            0.0, float(self.get_parameter("startup_online_ff_period_sec").value)
        )
        self._online_ff_cache: Optional[np.ndarray] = None
        self._online_ff_fz = np.zeros(4, dtype=float)
        self._online_ff_last_ns = 0
        self._r_wb = np.eye(3)
        if bool(self.get_parameter("startup_online_ff_enabled").value):
            urdf_xml = str(self.get_parameter("robot_description").value)
            if urdf_xml.strip():
                try:
                    self._online_ff = QuasiStaticStandupFF(urdf_xml)
                    self.get_logger().info(
                        "startup online quasi-static FF ready (mass=%.3f kg)"
                        % self._online_ff.total_mass
                    )
                except Exception as exc:
                    self.get_logger().warn(
                        f"startup online FF unavailable ({exc}); "
                        "falling back to constant feed-forward"
                    )
            else:
                self.get_logger().warn(
                    "startup_online_ff_enabled but robot_description is empty; "
                    "falling back to constant feed-forward"
                )

        self._joint_cmd: Optional[np.ndarray] = None
        self._rail_cmd: Optional[np.ndarray] = None
        self._joint_stamp_ns: Optional[int] = None
        self._rail_stamp_ns: Optional[int] = None
        self._first_joint_state_ns: Optional[int] = None
        self._hold_start_ns: Optional[int] = None
        self._handoff_start_ns: Optional[int] = None
        self._settle_ok_since_ns: Optional[int] = None
        self._trunk_up_z: Optional[float] = None
        self._trunk_pitch: Optional[float] = None
        self._odom_stamp_ns: Optional[int] = None
        self._forced_handoff = False
        self._subinfo_fallback_warned = False
        self._crossing_state = "UNKNOWN"
        self._published_safe_zero = False
        self._last_debug_log_ns: int = 0

        self._joint_sub = self.create_subscription(
            Float64MultiArray, self._joint_effort_topic, self._on_joint_effort, 10
        )
        self._rail_sub = self.create_subscription(
            Float64MultiArray, self._rail_effort_topic, self._on_rail_effort, 10
        )
        self._joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )
        self._crossing_state_sub = self.create_subscription(
            String, self._crossing_state_topic, self._on_crossing_state, 10
        )
        self._odom_sub = self.create_subscription(
            Odometry, handoff_odom_topic, self._on_odom, 10
        )
        self._pub = self.create_publisher(Float64MultiArray, self._output_topic, 10)
        self._timer = self.create_timer(max(1.0 / publish_rate_hz, 0.001), self._on_timer)

        self.get_logger().info(
            "wbc_effort_mux ready: joint='%s', rail='%s', output='%s' (%d ch), startup_hold=%s, "
            "hold_sec=%.2f, ramp_sec=%.2f, "
            "freeze_rail_effort=%s, freeze_hover=%s, "
            "freeze_walking=%s, freeze_staging=%s, freeze_flat=%s, state_topic='%s'"
            % (
                self._joint_effort_topic,
                self._rail_effort_topic,
                self._output_topic,
                self._output_size(),
                self._startup_hold_enabled,
                self._startup_hold_sec,
                self._startup_ramp_sec,
                self._freeze_rail_effort,
                self._freeze_rail_effort_in_hover,
                self._freeze_rail_effort_in_walking,
                self._freeze_rail_effort_in_crossing_staging,
                self._freeze_rail_effort_on_flat,
                self._crossing_state_topic,
            )
        )

    def _now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _on_joint_effort(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != 12:
            self.get_logger().error(f"Expected 12 WBC joint efforts, got {len(msg.data)}")
            return
        self._joint_cmd = np.asarray(msg.data, dtype=float)
        self._joint_stamp_ns = self._now_ns()

    def _on_rail_effort(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != 4:
            self.get_logger().error(f"Expected 4 WBC rail efforts, got {len(msg.data)}")
            return
        self._rail_cmd = np.asarray(msg.data, dtype=float)
        self._rail_stamp_ns = self._now_ns()

    def _on_joint_state(self, msg: JointState) -> None:
        if self._first_joint_state_ns is None and msg.name:
            self._first_joint_state_ns = self._now_ns()
            self.get_logger().info("startup hold timer started from first /joint_states")

        seen = 0
        for i, name in enumerate(msg.name):
            rail_idx = self._rail_index.get(name)
            if rail_idx is not None and i < len(msg.position):
                self._rail_pos[rail_idx] = msg.position[i]
                continue
            idx = self._joint_index.get(name)
            if idx is None:
                continue
            if i < len(msg.position):
                self._joint_pos[idx] = msg.position[i]
            if i < len(msg.velocity):
                self._joint_vel[idx] = msg.velocity[i]
            seen += 1
        if seen == 12:
            self._joint_state_valid = True

    def _on_crossing_state(self, msg: String) -> None:
        self._crossing_state = str(msg.data)

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        # z-component of the body z-axis in world: 1 level, -1 upside down.
        self._trunk_up_z = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        # ZYX pitch, yaw-independent: sin(pitch) = -(body x-axis).z.
        # pitch > 0 means the +X (tail / hind feet) end points down.
        self._trunk_pitch = math.asin(
            float(np.clip(2.0 * (q.w * q.y - q.x * q.z), -1.0, 1.0))
        )
        self._r_wb = _quat_to_rot(q.w, q.x, q.y, q.z)
        self._odom_stamp_ns = self._now_ns()

    def _odom_fresh(self) -> bool:
        if self._odom_stamp_ns is None:
            return False
        return (self._now_ns() - self._odom_stamp_ns) / 1e9 <= 1.0

    def _trunk_tilt_rad(self) -> Optional[float]:
        if self._trunk_up_z is None or not self._odom_fresh():
            return None
        return math.acos(float(np.clip(self._trunk_up_z, -1.0, 1.0)))

    def _trunk_pitch_rad(self) -> Optional[float]:
        if self._trunk_pitch is None or not self._odom_fresh():
            return None
        return self._trunk_pitch

    def _controller_subscribed(self) -> bool:
        if not self._output_controller_node:
            return self._pub.get_subscription_count() > 0
        try:
            infos = self.get_subscriptions_info_by_topic(self._output_topic)
        except Exception as exc:
            if not self._subinfo_fallback_warned:
                self._subinfo_fallback_warned = True
                self.get_logger().warn(
                    f"get_subscriptions_info_by_topic failed ({exc}); "
                    "falling back to raw subscription count"
                )
            return self._pub.get_subscription_count() > 0
        return any(
            info.node_name == self._output_controller_node for info in infos
        )

    def _is_fresh(self, stamp_ns: Optional[int]) -> bool:
        if stamp_ns is None:
            return False
        age_sec = max(0.0, (self._now_ns() - stamp_ns) / 1e9)
        return age_sec <= self._command_timeout_sec

    @staticmethod
    def _compose_effort_command(joint_cmd: np.ndarray, rail_cmd: np.ndarray) -> list[float]:
        effort = np.zeros(16, dtype=float)
        for leg in range(4):
            effort[leg * 4 + 0] = rail_cmd[leg]
            effort[leg * 4 + 1 : leg * 4 + 4] = joint_cmd[leg * 3 : (leg + 1) * 3]
        return effort.tolist()

    def _compose_output(self, joint_cmd: np.ndarray, rail_cmd: np.ndarray) -> list[float]:
        if self._include_rail_in_output:
            return self._compose_effort_command(joint_cmd, rail_cmd)
        # 12-channel layout: [coxa, femur, tibia] x [lf, lh, rh, rf], matching
        # the rail-less effort_controller joint list.
        return np.asarray(joint_cmd, dtype=float).tolist()

    def _output_size(self) -> int:
        return 16 if self._include_rail_in_output else 12

    def _publish(self, data: list[float]) -> None:
        msg = Float64MultiArray()
        msg.data = data
        self._pub.publish(msg)

    def _startup_elapsed_sec(self) -> Optional[float]:
        if not self._startup_hold_enabled:
            return None
        if self._first_joint_state_ns is None:
            self.get_logger().warn(
                "startup hold waiting for /joint_states...",
                throttle_duration_sec=2.0,
            )
            return None
        # The hold window must cover the time the effort controller can
        # actually act, not the time joint states first appeared: the
        # controller is spawned several seconds after the broadcaster and
        # everything published before it subscribes is dropped.
        if self._hold_start_ns is None:
            if not self._controller_subscribed():
                self.get_logger().info(
                    "startup hold waiting for effort controller subscriber "
                    f"(node '{self._output_controller_node}')...",
                    throttle_duration_sec=2.0,
                )
                return 0.0
            self._hold_start_ns = self._now_ns()
            # Restart the pose ramp from the pose measured right now: the
            # robot kept collapsing while the controller chain came up, so
            # an initial pose captured earlier no longer matches the heap
            # the PD is about to act on.
            if self._joint_state_valid:
                self._standup_initial_pose = self._joint_pos.copy()
            self.get_logger().info(
                "startup hold timer started (controller subscriber present)"
            )
        return max(0.0, (self._now_ns() - self._hold_start_ns) / 1e9)

    def _standup_level_deltas(self, s: float) -> np.ndarray:
        """Differential femur/tibia target offsets that level the trunk.

        pitch > 0 = tail (+X, hind feet) down = hind legs too collapsed.
        Collapse moves femur above and tibia below the stance target, so
        extending a leg means femur -= delta, tibia += delta. Hind legs
        extend and front legs fold by the same amount; the offset fades in
        with the pose ramp (s) and vanishes as the trunk levels out.
        """
        deltas = np.zeros(12, dtype=float)
        if not self._standup_level_enabled or s <= 0.0:
            return deltas
        pitch = self._trunk_pitch_rad()
        if pitch is None:
            return deltas
        delta = float(
            np.clip(
                self._standup_level_kp * pitch * s,
                -self._standup_level_max_delta,
                self._standup_level_max_delta,
            )
        )
        # Leg order [lf, lh, rh, rf] = [front, hind, hind, front].
        for leg, front in enumerate((True, False, False, True)):
            sign = 1.0 if front else -1.0
            deltas[leg * 3 + 1] = sign * delta   # femur
            deltas[leg * 3 + 2] = -sign * delta  # tibia
        return deltas

    def _online_ff_vector(self) -> Optional[np.ndarray]:
        """周期性重算在线前馈；异常时永久回退常量前馈。"""
        if self._online_ff is None:
            return None
        now_ns = self._now_ns()
        if (
            self._online_ff_cache is not None
            and self._online_ff_period_sec > 0.0
            and (now_ns - self._online_ff_last_ns) / 1e9 < self._online_ff_period_sec
        ):
            return self._online_ff_cache
        try:
            r_wb = self._r_wb if self._odom_fresh() else np.eye(3)
            tau12, fz = self._online_ff.compute(
                self._joint_pos, self._rail_pos, r_wb
            )
        except Exception as exc:
            self._online_ff = None
            self.get_logger().error(
                f"startup online FF failed ({exc}); reverting to constant FF"
            )
            return None
        self._online_ff_cache = tau12
        self._online_ff_fz = fz
        self._online_ff_last_ns = now_ns
        return tau12

    def _startup_hold_output(self) -> np.ndarray:
        if self._standup_pd_enabled and self._joint_state_valid:
            if self._standup_initial_pose is None:
                self._standup_initial_pose = self._joint_pos.copy()
            elapsed = self._startup_elapsed_sec() or 0.0
            s = float(np.clip(elapsed / self._standup_pose_ramp_sec, 0.0, 1.0))
            target = self._standup_initial_pose + s * (
                self._standup_pose - self._standup_initial_pose
            )
            target = target + self._standup_level_deltas(s)
            ff_scale = s if self._hold_effort_ramp_enabled else 1.0
            ff_vector = self._online_ff_vector()
            if ff_vector is None:
                ff_vector = self._startup_hold_joint_effort
            joint_effort = np.clip(
                self._standup_kp * (target - self._joint_pos)
                - self._standup_kd * self._joint_vel
                + ff_scale * ff_vector,
                -self._standup_max_torque,
                self._standup_max_torque,
            )
            return np.asarray(
                self._compose_output(joint_effort, np.zeros(4, dtype=float)),
                dtype=float,
            )
        return self._startup_hold_effort

    def _handoff_ready(self, elapsed: float) -> bool:
        if elapsed < self._startup_hold_sec:
            return False
        if not self._handoff_require_settle:
            return True

        if elapsed >= self._startup_hold_sec + self._handoff_max_extra_sec:
            if not self._forced_handoff:
                self._forced_handoff = True
                tilt = self._trunk_tilt_rad()
                self.get_logger().error(
                    "startup handoff forced after %.1f s without settling "
                    "(tilt=%s, max|qd|=%.2f rad/s)"
                    % (
                        elapsed,
                        f"{tilt:.2f} rad" if tilt is not None else "n/a",
                        float(np.max(np.abs(self._joint_vel))),
                    )
                )
            return True

        tilt = self._trunk_tilt_rad()
        if tilt is None:
            # No attitude source (estimator absent or stale): degrade to the
            # plain timer after a grace period instead of deadlocking bringup
            # modes that run without the research state estimator.
            if elapsed >= self._startup_hold_sec + self._handoff_no_odom_grace_sec:
                self.get_logger().warn(
                    "startup handoff without attitude feedback (no fresh odom); "
                    "proceeding on timer",
                    throttle_duration_sec=5.0,
                )
                return True
            return False

        max_joint_speed = (
            float(np.max(np.abs(self._joint_vel)))
            if self._joint_state_valid
            else float("inf")
        )
        settled = (
            tilt <= self._handoff_tilt_max_rad
            and max_joint_speed <= self._handoff_joint_speed_max
        )
        now_ns = self._now_ns()
        if settled:
            if self._settle_ok_since_ns is None:
                self._settle_ok_since_ns = now_ns
            if (now_ns - self._settle_ok_since_ns) / 1e9 >= self._handoff_settle_duration_sec:
                return True
        else:
            self._settle_ok_since_ns = None

        pitch = self._trunk_pitch_rad()
        self.get_logger().info(
            "startup handoff waiting for settle: tilt=%.3f rad (max %.3f), "
            "pitch=%s, max|qd|=%.2f rad/s (max %.2f)"
            % (
                tilt,
                self._handoff_tilt_max_rad,
                f"{pitch:+.3f}" if pitch is not None else "n/a",
                max_joint_speed,
                self._handoff_joint_speed_max,
            ),
            throttle_duration_sec=2.0,
        )
        return False

    def _publish_startup_hold_if_needed(self) -> bool:
        if not self._startup_hold_enabled:
            return False

        elapsed = self._startup_elapsed_sec()
        if elapsed is None:
            return True

        if self._handoff_start_ns is None:
            if not self._handoff_ready(elapsed):
                self._publish(self._startup_hold_output().tolist())
                if self._online_ff is not None and self._online_ff_cache is not None:
                    fz = self._online_ff_fz
                    self.get_logger().info(
                        "startup hold stand-up PD + online FF "
                        "fz=[%.1f %.1f %.1f %.1f] N" % (fz[0], fz[1], fz[2], fz[3]),
                        throttle_duration_sec=1.0,
                    )
                else:
                    self.get_logger().info(
                        "startup hold publishing stand-up PD effort (constant FF)",
                        throttle_duration_sec=1.0,
                    )
                return True
            self._handoff_start_ns = self._now_ns()
            tilt = self._trunk_tilt_rad()
            self.get_logger().info(
                "startup handoff: ramping to WBC output over %.1f s "
                "(hold took %.1f s, tilt=%s)"
                % (
                    self._startup_ramp_sec,
                    elapsed,
                    f"{tilt:.3f} rad" if tilt is not None else "n/a",
                )
            )
        return False

    def _apply_startup_ramp(self, effort: np.ndarray) -> np.ndarray:
        if self._handoff_start_ns is None or self._startup_ramp_sec <= 0.0:
            return effort

        ramp_elapsed = (self._now_ns() - self._handoff_start_ns) / 1e9
        if ramp_elapsed >= self._startup_ramp_sec:
            return effort

        alpha = ramp_elapsed / self._startup_ramp_sec
        self.get_logger().info(
            f"startup ramp alpha={float(np.clip(alpha, 0.0, 1.0)):.2f}",
            throttle_duration_sec=1.0,
        )
        return _blend_effort(self._startup_hold_output(), effort, alpha)

    def _should_freeze_rail_effort(self) -> bool:
        if self._freeze_rail_effort:
            return True
        if self._freeze_rail_effort_on_flat and self._crossing_state in (
            "HOVER",
            "WALKING",
            "FLAT_GAIT_DEMO",
        ):
            return True
        if self._freeze_rail_effort_in_hover and self._crossing_state == "HOVER":
            return True
        if self._freeze_rail_effort_in_walking and self._crossing_state == "WALKING":
            return True
        if (
            self._freeze_rail_effort_in_crossing_staging
            and self._crossing_state
            in ("CROSSING:PRE_APPROACH", "CROSSING:APPROACH")
        ):
            return True
        return False

    def _log_effort_breakdown(self, effort: np.ndarray) -> None:
        if not self._debug_enabled:
            return

        now_ns = self._now_ns()
        if self._last_debug_log_ns and (
            (now_ns - self._last_debug_log_ns) / 1e9 < self._debug_log_period_sec
        ):
            return

        self._last_debug_log_ns = now_ns
        parts = []
        per_leg = 4 if self._include_rail_in_output else 3
        for leg, name in enumerate(self._LEG_NAMES):
            base = leg * per_leg
            if self._include_rail_in_output:
                rail = float(effort[base + 0])
                coxa = float(effort[base + 1])
                femur = float(effort[base + 2])
                tibia = float(effort[base + 3])
            else:
                rail = 0.0
                coxa = float(effort[base + 0])
                femur = float(effort[base + 1])
                tibia = float(effort[base + 2])
            support_abs = abs(femur) + abs(tibia)
            total_abs = abs(rail) + abs(coxa) + support_abs
            parts.append(
                f"{name}[rail={rail:.1f} c={coxa:.1f} f={femur:.1f} t={tibia:.1f} "
                f"ft_abs={support_abs:.1f} sum={total_abs:.1f}]"
            )

        self.get_logger().info("effort_mux: " + " ".join(parts))

    def _on_timer(self) -> None:
        if self._publish_startup_hold_if_needed():
            return

        rail_required = self._include_rail_in_output
        if self._joint_cmd is None or (rail_required and self._rail_cmd is None):
            self.get_logger().warn(
                "Waiting for WBC joint%s effort stream..."
                % ("/rail" if rail_required else ""),
                throttle_duration_sec=2.0,
            )
            return

        joint_fresh = self._is_fresh(self._joint_stamp_ns)
        rail_fresh = (not rail_required) or self._is_fresh(self._rail_stamp_ns)
        if joint_fresh and rail_fresh:
            if rail_required:
                rail_cmd = (
                    np.zeros_like(self._rail_cmd)
                    if self._should_freeze_rail_effort()
                    else self._rail_cmd
                )
            else:
                rail_cmd = np.zeros(4, dtype=float)
            effort = np.asarray(
                self._compose_output(self._joint_cmd, rail_cmd),
                dtype=float,
            )
            effort = self._apply_startup_ramp(effort)
            self._publish(effort.tolist())
            self._log_effort_breakdown(effort)
            self._published_safe_zero = False
            return

        stale_joint = not joint_fresh
        stale_rail = rail_required and not self._is_fresh(self._rail_stamp_ns)
        if stale_joint:
            self.get_logger().error(
                f"WBC joint effort stream stale (topic={self._joint_effort_topic})",
                throttle_duration_sec=1.0,
            )
        if stale_rail:
            self.get_logger().error(
                f"WBC rail effort stream stale (topic={self._rail_effort_topic})",
                throttle_duration_sec=1.0,
            )

        if self._publish_safe_zero_on_stale:
            if not self._published_safe_zero:
                self.get_logger().warn(
                    "safe zero enabled: publishing zero effort on stale stream",
                    throttle_duration_sec=2.0,
                )
                self._published_safe_zero = True
            effort = np.zeros(self._output_size(), dtype=float)
            self._publish(effort.tolist())
        else:
            self.get_logger().warn(
                "WBC effort streams stale, no safe zero published (exposing true behavior)",
                throttle_duration_sec=2.0,
            )


def main() -> None:
    rclpy.init()
    node = WBCEffortMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
