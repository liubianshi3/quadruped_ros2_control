"""dog2 主运动控制节点（rclpy）。

该节点串联步态生成、目标平滑、逆运动学与 ros2_control 命令发布，保持 4-DoF 单腿拓扑：
- rail 作为前置调度参数（模式/相位驱动），
- 3R（hip_roll/hip_pitch/knee_pitch）作为标准 3-DoF 在固定 rail 下求解。
"""

import time
import math
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
import json
from enum import Enum
from typing import Dict, Optional

from rcl_interfaces.msg import SetParametersResult

from .gait_generator import GaitGenerator, GaitConfig
from .kinematics_solver import create_kinematics_solver
from .trajectory_planner import TrajectoryPlanner
from .joint_controller import JointController
from .joint_names import PREFIX_TO_LEG_MAP, get_leg_joint_names
from .config_loader import ConfigLoader
from .leg_parameters import reload_leg_parameter_joint_limits_from_urdf


class LocomotionMode(Enum):
    """运动模式：平地3DoF（导轨锁定）/越障4DoF（导轨激活）。"""

    CRUISE_3DOF = "cruise_3dof"
    OBSTACLE_4DOF = "obstacle_4dof"


class SpiderRobotController(Node):
    """dog2 主控制器（50Hz 级别主循环）。"""

    def __init__(self, config_path=None):
        super().__init__("spider_robot_controller")
        self.get_logger().info("Initializing dog2 motion controller...")

        self.config_loader = ConfigLoader(config_path)
        self.config_loader.load()
        gait_config = self.config_loader.get_gait_config()

        self.ik_solver = create_kinematics_solver()
        self.ik_solver.configure_regularization(self.config_loader.get_ik_regularization())
        self._reload_joint_limits_from_urdf(force_reload=False)

        control_params = self.config_loader.get_control_params()
        self.control_period_sec = float(
            self.declare_parameter(
                "control_period_sec",
                1.0 / float(control_params.get("frequency", 50.0)),
            ).value
        )
        self.max_rail_velocity_mps = float(
            self.declare_parameter("max_rail_velocity_mps", float(control_params.get("max_rail_velocity", 0.08))).value
        )
        self.max_rail_velocity_mps = max(1e-6, self.max_rail_velocity_mps)
        self.rail_violation_count = {"lf": 0, "rf": 0, "lh": 0, "rh": 0}
        self.max_rail_violation_before_emergency = int(
            self.declare_parameter("max_rail_violation_before_emergency", 10).value
        )

        self.rail_alpha_rate_per_sec = float(self.declare_parameter("rail_alpha_rate_per_sec", 0.5).value)
        self.velocity_lpf_tau_sec = float(self.declare_parameter("velocity_lpf_tau_sec", 0.25).value)
        self.rail_lock_alpha_epsilon = float(self.declare_parameter("rail_lock_alpha_epsilon", 1e-3).value)
        self.rail_locked_position_m = float(self.declare_parameter("rail_locked_position_m", 0.0).value)

        self.emergency_descent_final_body_height_m = float(
            self.declare_parameter("emergency_descent_final_body_height_m", 0.10).value
        )

        self.trajectory_planner = TrajectoryPlanner()
        self.joint_controller = JointController(self)

        standing_pose = self.config_loader.get_config_data().get("standing_pose", None)
        if standing_pose is None:
            raise RuntimeError(
                "Missing 'standing_pose' in config. Provide per-leg (rail, hip_roll, hip_pitch, knee_pitch)."
            )
        self._standing_joint_positions_by_leg = {
            leg_id: (
                float(standing_pose[leg_id]["rail_m"]),
                float(standing_pose[leg_id]["hip_roll_rad"]),
                float(standing_pose[leg_id]["hip_pitch_rad"]),
                float(standing_pose[leg_id]["knee_pitch_rad"]),
            )
            for leg_id in ("lf", "lh", "rh", "rf")
        }
        nominal_positions = {
            leg_id: self.ik_solver.solve_fk(leg_id, joints)
            for leg_id, joints in self._standing_joint_positions_by_leg.items()
        }
        self.gait_generator = GaitGenerator(gait_config, nominal_positions)

        self.current_velocity = (0.0, 0.0, 0.0)
        self.target_velocity = (0.0, 0.0, 0.0)
        self.is_running = False
        self.is_stopping = False
        self.stop_start_time = 0.0
        self.last_time = time.time()

        self.is_ramping = False
        self.ramp_start_time = 0.0
        self.ramp_duration_sec = float(self.declare_parameter("standup_ramp_duration_sec", 2.0).value)
        self.standing_joint_angles = {}
        self.ramp_last_valid_commands = {}
        self._standup_start_joints: Optional[Dict[str, float]] = None

        self.is_emergency_mode = False
        self.emergency_start_time = 0.0
        self.emergency_descent_duration_sec = float(
            self.declare_parameter("emergency_descent_duration_sec", 3.0).value
        )

        self.last_valid_joint_positions = {"lf": None, "rf": None, "lh": None, "rh": None}
        self.ik_failure_count = {"lf": 0, "rf": 0, "lh": 0, "rh": 0}

        self.pending_config_update = None
        self.last_cycle_phase = 0.0

        self.mode = LocomotionMode.CRUISE_3DOF
        self.target_mode = LocomotionMode.CRUISE_3DOF
        self.rail_activation_alpha = 0.0
        self.mode_transition_duration = 1.0

        self.rail_activation_mode = False
        self.current_rail_alpha = 0.0

        self.debug_mode = False
        self.debug_publisher = self.create_publisher(String, "/spider_debug_info", 10)
        self.debug_publish_counter = 0
        self._startup_ready_topic = str(
            self.declare_parameter("startup_ready_topic", "/spider_startup_ready").value
        )
        self._gazebo_unpause_prepare_topic = str(
            self.declare_parameter("gazebo_unpause_prepare_topic", "/gazebo_unpause_prepare").value
        )
        self._gazebo_unpaused_topic = str(
            self.declare_parameter("gazebo_unpaused_topic", "/gazebo_unpaused").value
        )
        startup_ready_qos = QoSProfile(depth=1)
        startup_ready_qos.reliability = ReliabilityPolicy.RELIABLE
        startup_ready_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._startup_ready_pub = self.create_publisher(Bool, self._startup_ready_topic, startup_ready_qos)
        self._startup_ready_published = False
        self._startup_sync_performed = False
        self._gazebo_unpause_prepare_seen = False
        self._gazebo_unpaused = False
        self._joint_state_seq_at_unpause = -1
        self._rail_slip_monitoring_started = False

        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self._cmd_vel_callback, 10
        )
        self.mode_switch_sub = self.create_subscription(
            String, "/mode_switch", self._mode_switch_callback, 10
        )
        self.rail_mode_sub = self.create_subscription(
            Bool, "/spider_rail_mode", self._on_rail_mode_msg, 10
        )
        self._gazebo_unpause_prepare_sub = self.create_subscription(
            Bool, self._gazebo_unpause_prepare_topic, self._on_gazebo_unpause_prepare, startup_ready_qos
        )
        self._gazebo_unpaused_sub = self.create_subscription(
            Bool, self._gazebo_unpaused_topic, self._on_gazebo_unpaused, startup_ready_qos
        )

        self.timer_period = float(self.declare_parameter("timer_period_sec", self.control_period_sec).value)
        self.timer = None

        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info(
            "Spider Robot Controller is READY. "
            "Mode=CRUISE_3DOF(alpha=0.0). Use /mode_switch with 'cruise_3dof' or 'obstacle_4dof'."
        )

    def _on_set_parameters(self, params):
        for p in params:
            if p.name == "rail_alpha_rate_per_sec":
                self.rail_alpha_rate_per_sec = float(p.value)
            elif p.name == "velocity_lpf_tau_sec":
                self.velocity_lpf_tau_sec = float(p.value)
            elif p.name == "rail_lock_alpha_epsilon":
                self.rail_lock_alpha_epsilon = float(p.value)
            elif p.name == "rail_locked_position_m":
                self.rail_locked_position_m = float(p.value)
            elif p.name == "emergency_descent_duration_sec":
                self.emergency_descent_duration_sec = float(p.value)
            elif p.name == "emergency_descent_final_body_height_m":
                self.emergency_descent_final_body_height_m = float(p.value)
            elif p.name == "max_rail_velocity_mps":
                self.max_rail_velocity_mps = max(1e-6, float(p.value))
        return SetParametersResult(successful=True)

    def _cmd_vel_callback(self, msg):
        if abs(msg.linear.x) < 0.001 and abs(msg.linear.y) < 0.001 and abs(msg.angular.z) < 0.001:
            if not self.is_stopping and self.is_running:
                self.initiate_smooth_stop()
        else:
            self.is_stopping = False
            v = (msg.linear.x, msg.linear.y, msg.angular.z)
            self.target_velocity = v

    def _mode_switch_callback(self, msg: String):
        mode_raw = msg.data.strip().lower()
        if mode_raw in ("cruise", "cruise_3dof", "3dof", "flat"):
            self.target_mode = LocomotionMode.CRUISE_3DOF
        elif mode_raw in ("obstacle", "obstacle_4dof", "4dof", "window"):
            self.target_mode = LocomotionMode.OBSTACLE_4DOF
        else:
            self.get_logger().warn(
                f"Unknown mode_switch '{msg.data}'. Use cruise_3dof or obstacle_4dof."
            )
            return

        self.get_logger().info(
            f"Mode switch requested: {self.mode.value} -> {self.target_mode.value}."
        )

    def _on_rail_mode_msg(self, msg: Bool):
        try:
            self.rail_activation_mode = bool(getattr(msg, "data", False))
        except Exception:
            self.rail_activation_mode = False

    def _on_gazebo_unpause_prepare(self, msg: Bool):
        if not bool(getattr(msg, "data", False)):
            return
        if self._gazebo_unpause_prepare_seen:
            return
        self._gazebo_unpause_prepare_seen = True
        self.joint_controller.log_rail_debug_snapshot("before_unpause")

    def _on_gazebo_unpaused(self, msg: Bool):
        if not bool(getattr(msg, "data", False)):
            return
        if self._gazebo_unpaused:
            return
        self._gazebo_unpaused = True
        self._joint_state_seq_at_unpause = self.joint_controller.get_joint_state_seq()
        self.joint_controller.disable_rail_slip_monitoring(
            freeze_expected_targets=True,
            reset_counters=True,
        )
        self.get_logger().info(
            "Gazebo unpause acknowledged; waiting for the first fresh joint_states frame before enabling rail slip monitoring."
        )

    def start(self):
        self.get_logger().info("Starting 50Hz control loop on system clock (startup-safe while Gazebo is paused)...")
        self.is_running = True
        self.is_stopping = False
        self.last_time = time.time()
        self.joint_controller.disable_rail_slip_monitoring(
            freeze_expected_targets=False,
            reset_counters=True,
        )

        self._compute_standing_joint_angles()

        self.is_ramping = True
        self.ramp_start_time = time.time()
        self.ramp_last_valid_commands = {}
        self._standup_start_joints = None

        # Use a wall-clock timer so startup prearm can complete even when Gazebo
        # launches paused and /clock has not begun advancing yet.
        self.timer = self.create_timer(
            self.timer_period,
            self._timer_callback,
            clock=Clock(clock_type=ClockType.SYSTEM_TIME),
        )

    def _compute_standing_joint_angles(self):
        self.standing_joint_angles = {}
        for leg_id, (s_m, hip_roll_rad, hip_pitch_rad, knee_pitch_rad) in self._standing_joint_positions_by_leg.items():
            leg_num = PREFIX_TO_LEG_MAP[leg_id]
            jn = get_leg_joint_names(leg_num)
            self.standing_joint_angles[jn["rail"]] = s_m
            self.standing_joint_angles[jn["hip_roll"]] = hip_roll_rad
            self.standing_joint_angles[jn["hip_pitch"]] = hip_pitch_rad
            self.standing_joint_angles[jn["knee_pitch"]] = knee_pitch_rad
        self.get_logger().info(
            f"Standing joint angles computed for {len(self.standing_joint_angles)} joints."
        )

    def _execute_standup_trajectory(self):
        if not self.standing_joint_angles:
            self.get_logger().warn("Ramp: standing_joint_angles empty. Holding last valid commands.")
            if self.ramp_last_valid_commands:
                self.joint_controller.send_joint_commands(self.ramp_last_valid_commands)
            return

        if self._standup_start_joints is None:
            if not self.joint_controller.has_received_joint_states():
                return
            self._standup_start_joints = {}
            for jn, tgt in self.standing_joint_angles.items():
                st = self.joint_controller.current_joint_states.get(jn)
                if st is not None:
                    self._standup_start_joints[jn] = float(st["position"])
                else:
                    self._standup_start_joints[jn] = float(tgt)
            self.ramp_start_time = time.time()
            self.get_logger().info(
                "Standup baseline captured from joint_states (revolute: spawn->standing; rails: hold at snapshot)."
            )

        elapsed = time.time() - self.ramp_start_time
        t = min(elapsed / max(self.ramp_duration_sec, 1e-6), 1.0)
        phi = self.trajectory_planner.smooth_phase(t)

        assert self._standup_start_joints is not None
        # 起立阶段不要强行把 prismatic 插值到 YAML 的 rail_m=0：仿真里重力平衡轨位常非零，
        # 与 3R 同时猛拉易导致整机「出生即塌」。导轨保持初始测量；CRUISE 步态仍跟测量轨。
        commands = {}
        for jn in self.standing_joint_angles:
            tgt = float(self.standing_joint_angles[jn])
            start = float(self._standup_start_joints[jn])
            if jn.endswith("_rail_joint"):
                commands[jn] = start
            else:
                commands[jn] = start + phi * (tgt - start)
        self.ramp_last_valid_commands = commands.copy()
        self.joint_controller.send_joint_commands(commands)

        if t >= 1.0:
            self.is_ramping = False
            self.joint_controller.sync_rail_targets_from_joint_states()
            self._startup_sync_performed = True
            self.joint_controller.disable_rail_slip_monitoring(
                freeze_expected_targets=True,
                reset_counters=True,
            )
            self.joint_controller.log_rail_debug_snapshot("ready_before_publish")
            self._publish_startup_ready_once()
            self.get_logger().info("Standup trajectory complete. Entering normal gait loop.")

    def _publish_startup_ready_once(self) -> None:
        if self._startup_ready_published:
            return

        msg = Bool()
        msg.data = True
        self._startup_ready_pub.publish(msg)
        self._startup_ready_published = True
        self.get_logger().info(
            f"Startup ready signal published on {self._startup_ready_topic}."
        )

    def initiate_smooth_stop(self):
        self.get_logger().info("Initiating smooth stop...")
        self.is_stopping = True
        self.stop_start_time = self.gait_generator.current_time
        self.target_velocity = self.current_velocity

    def stop(self):
        self.get_logger().info("Stopping control loop, engaging safety lock...")
        self.is_running = False
        if self.timer is not None:
            self.timer.cancel()
        self.joint_controller.lock_rails_with_max_effort()

    def engage_emergency_safety_posture(self):
        self.get_logger().error("EMERGENCY: Engaging safety posture!")
        self.is_emergency_mode = True
        self.emergency_start_time = time.time()
        self.current_velocity = (0.0, 0.0, 0.0)
        self.target_velocity = (0.0, 0.0, 0.0)
        self.is_stopping = False
        self.joint_controller.lock_rails_with_max_effort()

    def _execute_emergency_descent(self):
        elapsed = time.time() - self.emergency_start_time
        if elapsed >= self.emergency_descent_duration_sec:
            self.get_logger().info("Emergency descent completed.")
            return True

        progress = elapsed / max(self.emergency_descent_duration_sec, 1e-6)
        initial_h = self.gait_generator.config.body_height
        current_h = initial_h - (initial_h - self.emergency_descent_final_body_height_m) * progress

        target_joint_positions = {}
        for leg_id in ["lf", "rf", "lh", "rh"]:
            if self.last_valid_joint_positions[leg_id] is not None:
                fp = self.ik_solver.solve_fk(leg_id, self.last_valid_joint_positions[leg_id])
                fp_adj = (fp[0], fp[1], fp[2] + (initial_h - current_h))
                ik = self.ik_solver.solve_ik(leg_id, fp_adj)
                s_m, haa, hfe, kfe = ik if ik else self.last_valid_joint_positions[leg_id]
            else:
                s_m, haa, hfe, kfe = 0.0, 0.0, 0.0, 0.0
            jn = get_leg_joint_names(PREFIX_TO_LEG_MAP[leg_id])
            target_joint_positions[jn["rail"]] = s_m
            target_joint_positions[jn["hip_roll"]] = haa
            target_joint_positions[jn["hip_pitch"]] = hfe
            target_joint_positions[jn["knee_pitch"]] = kfe

        self.joint_controller.send_joint_commands(target_joint_positions)
        if not self.joint_controller.monitor_rail_positions():
            self.get_logger().error("CRITICAL: Rail slip during emergency descent!")
        return False

    def _timer_callback(self):
        if not self.is_running:
            return
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        self.update(dt)

    def _maybe_enable_rail_slip_monitoring_after_unpause(self) -> None:
        if self._rail_slip_monitoring_started:
            return
        if self.is_ramping:
            return
        if not self._startup_sync_performed:
            return
        if not self._startup_ready_published:
            return
        if not self.joint_controller.has_received_joint_states():
            return
        if not self._gazebo_unpaused:
            return

        current_seq = self.joint_controller.get_joint_state_seq()
        if current_seq <= self._joint_state_seq_at_unpause:
            return

        self.joint_controller.log_rail_debug_snapshot("after_unpause_first_joint_state")
        self.joint_controller.sync_rail_targets_from_joint_states()
        self.joint_controller.enable_rail_slip_monitoring(reset_counters=True)
        self._rail_slip_monitoring_started = True
        self.get_logger().info(
            "Rail slip monitoring enabled after sync_rail_targets_from_joint_states -> unpause -> first fresh joint_states."
        )

    def update(self, dt):
        try:
            dt_eff = float(dt)
        except Exception:
            dt_eff = 0.0
        if not math.isfinite(dt_eff) or dt_eff <= 0.0:
            dt_eff = 0.0

        try:
            target_alpha = 1.0 if self.rail_activation_mode else 0.0
            step = float(self.rail_alpha_rate_per_sec) * dt_eff
            alpha = float(self.current_rail_alpha)
            if target_alpha > alpha:
                alpha = min(alpha + step, target_alpha)
            else:
                alpha = max(alpha - step, target_alpha)
            self.current_rail_alpha = max(0.0, min(1.0, alpha))
        except Exception:
            self.current_rail_alpha = max(0.0, min(1.0, float(getattr(self, "current_rail_alpha", 0.0) or 0.0)))

        if self.is_ramping:
            self._execute_standup_trajectory()
            return

        self._maybe_enable_rail_slip_monitoring_after_unpause()

        if not self.joint_controller.check_connection():
            self.get_logger().warn("Joint controller connection lost. Attempting reconnect...")
            if not self.joint_controller.attempt_reconnect():
                self.get_logger().error("Reconnection failed. Engaging emergency safety posture.")
                self.engage_emergency_safety_posture()
            return

        if self.is_emergency_mode:
            if self._execute_emergency_descent():
                self.stop()
            return

        self._check_and_apply_config_update()

        if not self.joint_controller.monitor_rail_positions():
            self.get_logger().error("CRITICAL: Rail slip detected!")
            self.engage_emergency_safety_posture()
            return

        stuck_joints = self.joint_controller.detect_stuck_joints()
        stuck_count = sum(1 for v in stuck_joints.values() if v)
        for jn, stuck in stuck_joints.items():
            if stuck:
                self.joint_controller.handle_stuck_joint(jn)
        if stuck_count >= 3:
            self.get_logger().error(f"CRITICAL: {stuck_count} joints stuck!")
            self.engage_emergency_safety_posture()
            return

        if self.is_stopping:
            self._handle_smooth_stop()
        else:
            alpha = min(1.0, dt / max(self.velocity_lpf_tau_sec, 1e-6)) if dt > 0.0 else 1.0
            cvx, cvy, comega = self.current_velocity
            tvx, tvy, tomega = self.target_velocity
            self.current_velocity = (
                cvx + alpha * (tvx - cvx),
                cvy + alpha * (tvy - cvy),
                comega + alpha * (tomega - comega),
            )

        self.gait_generator.update(dt, self.current_velocity)

        self._update_mode_transition(dt)

        target_joint_positions = {}
        for leg_id in ["lf", "rf", "lh", "rh"]:
            foot_pos = self.gait_generator.get_foot_target(leg_id)
            phase_scalar = self.gait_generator.get_phase_progress_scalar(leg_id)
            leg_params = self.ik_solver.leg_params[leg_id]
            rail_min, rail_max = leg_params.joint_limits['rail']
            rail_mid = 0.5 * (rail_min + rail_max)
            rail_half_span = 0.5 * (rail_max - rail_min)
            cruise_rail_locked = self.mode == LocomotionMode.CRUISE_3DOF and float(
                self.current_rail_alpha
            ) <= self.rail_lock_alpha_epsilon
            # CRUISE 锁轨：用当前测量的 prismatic 作为固定 rail 解 3R，并下发同一 rail。
            # 若强行 rail_hint=0 而仿真在重力下停在非零位，会出现数十毫米“假滑移”急停。
            if cruise_rail_locked:
                jn_pre = get_leg_joint_names(PREFIX_TO_LEG_MAP[leg_id])
                rail_joint = jn_pre["rail"]
                st = self.joint_controller.current_joint_states.get(rail_joint)
                if st is not None:
                    rail_hint = float(max(rail_min, min(rail_max, st["position"])))
                else:
                    rail_hint = float(self.rail_locked_position_m)
            else:
                rail_hint = rail_mid + (phase_scalar * rail_half_span) * self.current_rail_alpha

            ik_result = self.ik_solver.solve_ik(leg_id, foot_pos, rail_offset=rail_hint)

            if ik_result is None:
                self._handle_ik_failure(leg_id, foot_pos)
                if self.last_valid_joint_positions[leg_id] is not None:
                    s_m, haa, hfe, kfe = self.last_valid_joint_positions[leg_id]
                else:
                    self.get_logger().error(f"No last valid config for {leg_id}, using zero.")
                    s_m, haa, hfe, kfe = 0.0, 0.0, 0.0, 0.0
            else:
                s_m, haa, hfe, kfe = ik_result
                if not cruise_rail_locked:
                    s_m = self._apply_rail_velocity_limit(leg_id, s_m, dt)
                self.last_valid_joint_positions[leg_id] = (s_m, haa, hfe, kfe)
                self.ik_failure_count[leg_id] = 0

            jn = get_leg_joint_names(PREFIX_TO_LEG_MAP[leg_id])
            target_joint_positions[jn["rail"]] = s_m
            target_joint_positions[jn["hip_roll"]] = haa
            target_joint_positions[jn["hip_pitch"]] = hfe
            target_joint_positions[jn["knee_pitch"]] = kfe

        self.joint_controller.send_joint_commands(target_joint_positions)
        if self.debug_mode:
            self._publish_debug_info(target_joint_positions)

    def _update_mode_transition(self, dt: float):
        """平滑更新导轨激活因子 alpha∈[0,1]。"""
        target_alpha = 1.0 if self.target_mode == LocomotionMode.OBSTACLE_4DOF else 0.0
        if dt <= 0.0:
            self.rail_activation_alpha = target_alpha
        else:
            step = dt / max(self.mode_transition_duration, 1e-3)
            if target_alpha > self.rail_activation_alpha:
                self.rail_activation_alpha = min(target_alpha, self.rail_activation_alpha + step)
            else:
                self.rail_activation_alpha = max(target_alpha, self.rail_activation_alpha - step)

        if self.rail_activation_alpha <= 1e-3:
            new_mode = LocomotionMode.CRUISE_3DOF
        elif self.rail_activation_alpha >= 1.0 - 1e-3:
            new_mode = LocomotionMode.OBSTACLE_4DOF
        else:
            new_mode = self.mode

        if new_mode != self.mode:
            self.mode = new_mode
            self.get_logger().info(
                f"Mode reached: {self.mode.value}, rail_alpha={self.rail_activation_alpha:.2f}"
            )

    def _apply_rail_velocity_limit(self, leg_id, rail_target, dt):
        if dt <= 0.0:
            return rail_target

        last = self.last_valid_joint_positions.get(leg_id)
        if last is None:
            self.rail_violation_count[leg_id] = 0
            return rail_target

        prev_s = float(last[0])
        max_delta = self.max_rail_velocity_mps * dt
        delta = float(rail_target - prev_s)

        if abs(delta) <= max_delta:
            self.rail_violation_count[leg_id] = 0
            return rail_target

        limited_target = prev_s + max(-max_delta, min(max_delta, delta))
        self.rail_violation_count[leg_id] += 1
        self.get_logger().warn(
            f"Rail velocity limit hit on {leg_id}: "
            f"requested_delta={delta:.4f}m, limited_delta={limited_target - prev_s:.4f}m, dt={dt:.4f}s"
        )

        if (
            self.rail_violation_count[leg_id] > self.max_rail_violation_before_emergency
            and self.ik_failure_count[leg_id] > 3
        ):
            self.get_logger().error(
                f"CRITICAL: persistent rail velocity violation on {leg_id} "
                f"(count={self.rail_violation_count[leg_id]}, ik_fail={self.ik_failure_count[leg_id]})."
            )
            self.engage_emergency_safety_posture()

        return limited_target

    def _handle_smooth_stop(self):
        elapsed = self.gait_generator.current_time - self.stop_start_time
        stop_dur = self.gait_generator.config.cycle_time
        if elapsed >= stop_dur:
            self.current_velocity = (0.0, 0.0, 0.0)
            self.is_stopping = False
            self.get_logger().info("Smooth stop completed.")
        else:
            f = 1.0 - elapsed / stop_dur
            vx, vy, om = self.target_velocity
            self.current_velocity = (vx * f, vy * f, om * f)

    def update_gait_config(self, new_config):
        self.pending_config_update = new_config
        self.get_logger().info(
            f"Config update requested: stride_length={new_config.stride_length}, "
            f"stride_height={new_config.stride_height}, cycle_time={new_config.cycle_time}"
        )

    def reload_config_from_file(self, config_path=None):
        if config_path is not None:
            self.config_loader = ConfigLoader(config_path)
        self.config_loader.load()
        self._reload_joint_limits_from_urdf(force_reload=True)

        # 关节限位的权威来源现在是 dog2 xacro/URDF；
        # 这里在配置重载后刷新 JointController 内部的 clamp 表，避免后续扩展时出现脱节。
        try:
            if hasattr(self.joint_controller, "reload_joint_limits"):
                self.joint_controller.reload_joint_limits()
        except Exception as exc:
            self.get_logger().warn(f"Failed to reload joint limits in JointController: {exc}")

        control_params = self.config_loader.get_control_params()
        self.max_rail_velocity_mps = float(control_params.get("max_rail_velocity", self.max_rail_velocity_mps))
        self.max_rail_velocity_mps = max(1e-6, self.max_rail_velocity_mps)

        self.update_gait_config(self.config_loader.get_gait_config())
        self.get_logger().info(f"Config reloaded from {self.config_loader.config_path}")

    def _reload_joint_limits_from_urdf(self, *, force_reload: bool) -> None:
        """Sync shared IK/controller joint limits from the authoritative xacro source."""

        source_path = reload_leg_parameter_joint_limits_from_urdf(force_reload=force_reload)
        sample_leg = self.ik_solver.leg_params["lf"]
        self.get_logger().info(
            "Loaded joint limits from URDF: "
            f"source='{source_path}', "
            f"coxa={sample_leg.joint_limits['coxa']}, "
            f"femur={sample_leg.joint_limits['femur']}, "
            f"tibia={sample_leg.joint_limits['tibia']}, "
            f"rail_by_leg={{lf:{self.ik_solver.leg_params['lf'].joint_limits['rail']}, "
            f"lh:{self.ik_solver.leg_params['lh'].joint_limits['rail']}, "
            f"rh:{self.ik_solver.leg_params['rh'].joint_limits['rail']}, "
            f"rf:{self.ik_solver.leg_params['rf'].joint_limits['rail']}}}"
        )

    def _check_and_apply_config_update(self):
        if self.pending_config_update is None or self.is_stopping:
            return
        phase = self.gait_generator.get_phase("lf")
        if self.last_cycle_phase > 0.9 and phase < 0.1:
            self.gait_generator.config = self.pending_config_update
            self.pending_config_update = None
            self.get_logger().info("Config update applied at cycle boundary.")
        self.last_cycle_phase = phase

    def enable_debug_mode(self, enable=True):
        self.debug_mode = enable
        self.get_logger().info(f"Debug mode {'ENABLED' if enable else 'DISABLED'}.")

    def _publish_debug_info(self, joint_positions):
        if not self.debug_mode:
            return
        self.debug_publish_counter += 1
        if self.debug_publish_counter < 10:
            return
        self.debug_publish_counter = 0

        debug_data = {
            "timestamp": self.gait_generator.current_time,
            "gait_config": {
                "stride_length": self.gait_generator.config.stride_length,
                "stride_height": self.gait_generator.config.stride_height,
                "cycle_time": self.gait_generator.config.cycle_time,
                "duty_factor": self.gait_generator.config.duty_factor,
                "body_height": self.gait_generator.config.body_height,
                "gait_type": self.gait_generator.config.gait_type,
            },
            "velocity": {
                "vx": self.current_velocity[0],
                "vy": self.current_velocity[1],
                "omega": self.current_velocity[2],
            },
            "legs": {},
        }
        for leg_id in ["lf", "rf", "lh", "rh"]:
            jn = get_leg_joint_names(PREFIX_TO_LEG_MAP[leg_id])
            fp = self.gait_generator.get_foot_target(leg_id)
            debug_data["legs"][leg_id] = {
                "phase": round(self.gait_generator.get_phase(leg_id), 3),
                "is_stance": self.gait_generator.is_stance_phase(leg_id),
                "foot_position": {"x": round(fp[0], 4), "y": round(fp[1], 4), "z": round(fp[2], 4)},
                "joint_positions": {
                    "rail_m": round(joint_positions.get(jn["rail"], 0.0), 4),
                    "hip_roll_rad": round(joint_positions.get(jn["hip_roll"], 0.0), 4),
                    "hip_pitch_rad": round(joint_positions.get(jn["hip_pitch"], 0.0), 4),
                    "knee_pitch_rad": round(joint_positions.get(jn["knee_pitch"], 0.0), 4),
                },
            }
        support_legs = self.gait_generator.get_support_triangle()
        debug_data["support_legs"] = support_legs
        debug_data["num_support_legs"] = len(support_legs)
        debug_data["is_stable"] = self.gait_generator.verify_stability()

        msg = String()
        msg.data = json.dumps(debug_data, indent=2)
        self.debug_publisher.publish(msg)

    def _handle_ik_failure(self, leg_id, target_pos):
        self.ik_failure_count[leg_id] += 1
        self.get_logger().error(
            f"IK Failure on {leg_id} at {target_pos}. "
            f"Using last valid config. (count: {self.ik_failure_count[leg_id]})"
        )
        if self.ik_failure_count[leg_id] > 10:
            self.get_logger().warn(
                f"Leg {leg_id} has {self.ik_failure_count[leg_id]} consecutive IK failures."
            )


def main(args=None):
    rclpy.init(args=args)
    controller = SpiderRobotController()
    try:
        controller.start()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Keyboard Interrupt detected.")
    finally:
        try:
            controller.stop()
        except Exception:
            pass
        try:
            controller.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
