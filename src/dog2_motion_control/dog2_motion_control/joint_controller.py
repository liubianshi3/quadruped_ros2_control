"""ros2_control 关节命令与状态桥接。

该模块只做两件事：
- 发布 16 通道位置指令（4 条腿 × (1 prismatic rail + 3 revolute)）
- 订阅 `/joint_states` 并提供安全监测（限位夹紧、导轨滑移、卡死检测）
"""

from __future__ import annotations

import time
from typing import Dict, Optional, Tuple

from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .joint_names import (
    LEG_PREFIX_MAP,
    get_joint_name,
    get_rail_joint_name,
)
from .leg_parameters import LEG_PARAMETERS


class JointController:
    """ros2_control 接口：发布轨迹、读取状态、运行安全监控。"""

    def __init__(self, node: Node):
        self.node = node

        self._revolute_controller_topic = str(
            node.declare_parameter(
                "revolute_controller_topic",
                "/joint_trajectory_controller/joint_trajectory",
            ).value
        )
        self._rail_controller_topic = str(
            node.declare_parameter(
                "rail_controller_topic",
                "/rail_position_controller/joint_trajectory",
            ).value
        )
        self._trajectory_time_from_start_sec = float(
            node.declare_parameter("trajectory_time_from_start_sec", 0.02).value
        )
        self._rail_hold_time_from_start_sec = float(
            node.declare_parameter("rail_hold_time_from_start_sec", 0.01).value
        )

        self._joint_state_topic = str(node.declare_parameter("joint_state_topic", "/joint_states").value)
        self._publisher_queue_depth = int(node.declare_parameter("publisher_queue_depth", 10).value)
        self._subscriber_queue_depth = int(node.declare_parameter("subscriber_queue_depth", 10).value)

        self._connection_timeout_sec = float(node.declare_parameter("connection_timeout_sec", 3.0).value)
        self._max_reconnect_attempts = int(node.declare_parameter("max_reconnect_attempts", 5).value)

        self._stuck_detection_consecutive_count = int(
            node.declare_parameter("stuck_detection_consecutive_count", 50).value
        )
        self._stuck_position_error_threshold = float(
            node.declare_parameter("stuck_position_error_threshold", 3.14).value
        )

        self.rails_enabled = bool(node.declare_parameter("rails_enabled", True).value)
        self._rail_slip_threshold_m = float(node.declare_parameter("rail_slip_threshold_m", 0.005).value)
        self._rail_slip_patience = max(1, int(node.declare_parameter("rail_slip_patience", 3).value))

        if self._trajectory_time_from_start_sec <= 0.0:
            raise ValueError("trajectory_time_from_start_sec must be positive.")
        if self._rail_hold_time_from_start_sec <= 0.0:
            raise ValueError("rail_hold_time_from_start_sec must be positive.")
        if self._connection_timeout_sec <= 0.0:
            raise ValueError("connection_timeout_sec must be positive.")

        self.revolute_trajectory_pub = node.create_publisher(
            JointTrajectory,
            self._revolute_controller_topic,
            self._publisher_queue_depth,
        )
        self.rail_trajectory_pub = node.create_publisher(
            JointTrajectory,
            self._rail_controller_topic,
            self._publisher_queue_depth,
        )

        self.commanded_rail_targets: Dict[str, float] = {}
        self.last_rail_targets: Dict[str, float] = {}

        self.joint_state_sub = node.create_subscription(
            JointState,
            self._joint_state_topic,
            self._joint_state_callback,
            self._subscriber_queue_depth,
        )

        self.current_joint_states: Dict[str, Dict[str, float]] = {}

        self.last_joint_state_time = node.get_clock().now()
        self.last_joint_state_wall_time = time.monotonic()
        self.is_connected = False
        self.reconnect_attempts = 0
        self._joint_state_seq = 0
        self._last_sync_joint_state_seq = -1

        self.joint_command_history: Dict[str, list] = {}
        self.joint_stuck_count: Dict[str, int] = {}

        self.rail_slip_counters: Dict[str, int] = {}
        self._rail_slip_monitoring_enabled = True
        self._freeze_expected_rail_targets = False

        self._load_joint_limits()

        self.node.get_logger().info(
            "JointController ready: rails_enabled=%s, joint_state_topic=%s, timeout=%.3fs"
            % (
                self.rails_enabled,
                self._joint_state_topic,
                self._connection_timeout_sec,
            )
        )
    
    def _load_joint_limits(self) -> None:
        try:
            self.joint_limits: Dict[str, Tuple[float, float]] = {}
            
            for leg_num in (1, 2, 3, 4):
                prefix = LEG_PREFIX_MAP[leg_num]
                leg_params = LEG_PARAMETERS[prefix]
                
                rail_joint = get_rail_joint_name(leg_num)
                self.joint_limits[rail_joint] = leg_params.joint_limits["rail"]
                
                for role in ("hip_roll", "hip_pitch", "knee_pitch"):
                    joint_name = get_joint_name(leg_num, role)
                    # leg_parameters 仍使用历史键名：coxa/femur/tibia
                    if role == "hip_roll":
                        limit_key = "coxa"
                    elif role == "hip_pitch":
                        limit_key = "femur"
                    else:
                        limit_key = "tibia"
                    self.joint_limits[joint_name] = leg_params.joint_limits[limit_key]

            self.node.get_logger().info(f"Loaded joint limits: {len(self.joint_limits)} joints")
        except Exception as e:
            self.node.get_logger().error(f"Failed to load joint limits: {e}")
            self.joint_limits = {}
            raise

    def sync_rail_targets_from_joint_states(self) -> None:
        """将导轨滑移监控的期望值对齐到当前关节状态（仿真中立起后常用）。

        避免「命令 rail=0」与「物理平衡位置非零」的长期偏差被误判为滑移。
        """
        for leg_num in (1, 2, 3, 4):
            rail_joint = get_rail_joint_name(leg_num)
            self.rail_slip_counters[rail_joint] = 0
            if rail_joint in self.current_joint_states:
                self.last_rail_targets[rail_joint] = float(
                    self.current_joint_states[rail_joint]["position"]
                )
        self._last_sync_joint_state_seq = self._joint_state_seq

    def disable_rail_slip_monitoring(self, freeze_expected_targets: bool = True, reset_counters: bool = False) -> None:
        self._rail_slip_monitoring_enabled = False
        self._freeze_expected_rail_targets = bool(freeze_expected_targets)
        if reset_counters:
            for leg_num in (1, 2, 3, 4):
                self.rail_slip_counters[get_rail_joint_name(leg_num)] = 0

    def enable_rail_slip_monitoring(self, reset_counters: bool = False) -> None:
        self._rail_slip_monitoring_enabled = True
        self._freeze_expected_rail_targets = False
        if reset_counters:
            for leg_num in (1, 2, 3, 4):
                self.rail_slip_counters[get_rail_joint_name(leg_num)] = 0

    def has_received_joint_states(self) -> bool:
        return self._joint_state_seq > 0

    def get_joint_state_seq(self) -> int:
        return int(self._joint_state_seq)

    @staticmethod
    def _format_rail_debug_mm(values: Dict[str, float]) -> Dict[str, float]:
        return {joint: round(float(value) * 1e3, 3) for joint, value in sorted(values.items())}

    def get_rail_debug_snapshot(self) -> Tuple[Dict[str, float], Dict[str, float], Dict[str, int]]:
        measured = {}
        expected = {}
        counters = {}
        for leg_num in (1, 2, 3, 4):
            rail_joint = get_rail_joint_name(leg_num)
            measured[rail_joint] = float(
                self.current_joint_states.get(rail_joint, {}).get("position", 0.0)
            )
            expected[rail_joint] = float(self.last_rail_targets.get(rail_joint, 0.0))
            counters[rail_joint] = int(self.rail_slip_counters.get(rail_joint, 0))
        return measured, expected, counters

    def log_rail_debug_snapshot(self, label: str) -> None:
        measured, expected, counters = self.get_rail_debug_snapshot()
        self.node.get_logger().info(
            "Rail debug [%s]: measured_rail=%s expected_rail=%s slip_counter=%s"
            % (
                label,
                self._format_rail_debug_mm(measured),
                self._format_rail_debug_mm(expected),
                counters,
            )
        )

    def reload_joint_limits(self) -> None:
        """从 LEG_PARAMETERS 刷新关节限位映射表。

        当前阶段 LEG_PARAMETERS 会由 dog2 xacro/URDF 同步生成；
        当模型更新后，可调用该方法在不重启节点的情况下刷新 clamp 范围。
        """
        try:
            self._load_joint_limits()
            self.node.get_logger().info("Joint limits reloaded from LEG_PARAMETERS.")
        except Exception as e:
            self.node.get_logger().error(f"Failed to reload joint limits: {e}")
    
    def _joint_state_callback(self, msg: JointState) -> None:
        self.last_joint_state_time = self.node.get_clock().now()
        self.last_joint_state_wall_time = time.monotonic()
        self._joint_state_seq += 1
        
        if not self.is_connected:
            self.is_connected = True
            self.reconnect_attempts = 0
            self.node.get_logger().info("Joint states received, controller connected.")
        
        for i, name in enumerate(msg.name):
            self.current_joint_states[name] = {
                'position': msg.position[i] if i < len(msg.position) else 0.0,
                'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
            }
    
    def send_joint_commands(self, joint_positions: Dict[str, float]) -> None:
        """发布 16 通道位置指令（rail: m，revolute: rad）。"""
        rail_trajectory = JointTrajectory()
        rail_point = JointTrajectoryPoint()

        for leg_num in (1, 2, 3, 4):
            rail_joint = get_joint_name(leg_num, "rail")
            rail_trajectory.joint_names.append(rail_joint)
            rail_target_m = float(joint_positions.get(rail_joint, 0.0))
            rail_target_m = self.check_joint_limits(rail_joint, rail_target_m)
            rail_point.positions.append(rail_target_m)
            self.commanded_rail_targets[rail_joint] = rail_target_m
            if not self._freeze_expected_rail_targets:
                self.last_rail_targets[rail_joint] = rail_target_m
            self._record_joint_command(rail_joint, rail_target_m)

        rail_point.time_from_start = Duration(
            sec=int(self._trajectory_time_from_start_sec),
            nanosec=int((self._trajectory_time_from_start_sec % 1.0) * 1e9),
        )
        rail_trajectory.points.append(rail_point)

        if self.rails_enabled and len(rail_trajectory.joint_names) > 0:
            self.rail_trajectory_pub.publish(rail_trajectory)

        revolute_trajectory = JointTrajectory()
        revolute_point = JointTrajectoryPoint()

        for leg_num in (1, 2, 3, 4):
            for role in ("hip_roll", "hip_pitch", "knee_pitch"):
                joint_name = get_joint_name(leg_num, role)
                revolute_trajectory.joint_names.append(joint_name)
                target_rad = float(joint_positions.get(joint_name, 0.0))
                target_rad = self.check_joint_limits(joint_name, target_rad)
                revolute_point.positions.append(target_rad)
                self._record_joint_command(joint_name, target_rad)

        revolute_point.time_from_start = Duration(
            sec=int(self._trajectory_time_from_start_sec),
            nanosec=int((self._trajectory_time_from_start_sec % 1.0) * 1e9),
        )
        revolute_trajectory.points.append(revolute_point)
        self.revolute_trajectory_pub.publish(revolute_trajectory)
    
    def get_joint_states(self) -> Dict[str, Dict[str, float]]:
        """获取最新的关节状态缓存。"""
        return self.current_joint_states
    
    def get_joint_position(self, joint_name: str) -> Optional[float]:
        """获取单个关节位置（不存在返回 None）。"""
        if joint_name in self.current_joint_states:
            return self.current_joint_states[joint_name]['position']
        return None
    
    def check_joint_limits(self, joint_name: str, position: float) -> float:
        if joint_name not in self.joint_limits:
            self.node.get_logger().warn(
                "Joint %s has no configured limits; sending raw target." % joint_name
            )
            return position
        
        lower_limit, upper_limit = self.joint_limits[joint_name]
        
        # 检查是否超限
        if position < lower_limit:
            is_rail = joint_name.endswith('_rail_joint')
            unit = 'm' if is_rail else 'rad'
            self.node.get_logger().warn(
                "Joint %s target %.6f%s below lower limit %.6f%s; clamped."
                % (joint_name, position, unit, lower_limit, unit)
            )
            return lower_limit
        
        if position > upper_limit:
            is_rail = joint_name.endswith('_rail_joint')
            unit = 'm' if is_rail else 'rad'
            self.node.get_logger().warn(
                "Joint %s target %.6f%s above upper limit %.6f%s; clamped."
                % (joint_name, position, unit, upper_limit, unit)
            )
            return upper_limit
        
        return position
    
    def monitor_rail_positions(self) -> bool:
        if not self.rails_enabled:
            return True
        if not self._rail_slip_monitoring_enabled:
            return True

        all_rails_ok = True
        
        for leg_num in (1, 2, 3, 4):
            rail_joint = get_rail_joint_name(leg_num)
            
            if rail_joint in self.current_joint_states:
                actual_pos_m = self.current_joint_states[rail_joint]['position']
                expected_pos_m = self.last_rail_targets.get(rail_joint, actual_pos_m)
                track_error_m = actual_pos_m - expected_pos_m

                if abs(track_error_m) > self._rail_slip_threshold_m:
                    count = self.rail_slip_counters.get(rail_joint, 0) + 1
                    self.rail_slip_counters[rail_joint] = count
                    if count >= self._rail_slip_patience:
                        self.node.get_logger().error(
                            "Rail tracking error on %s: %.3f mm (threshold=%.3f mm, count=%d/%d)"
                            % (
                                rail_joint,
                                track_error_m * 1e3,
                                self._rail_slip_threshold_m * 1e3,
                                count,
                                self._rail_slip_patience,
                            )
                        )
                        all_rails_ok = False
                    else:
                        self.node.get_logger().warn(
                            "Rail transient error on %s: %.3f mm (threshold=%.3f mm, count=%d/%d)"
                            % (
                                rail_joint,
                                track_error_m * 1e3,
                                self._rail_slip_threshold_m * 1e3,
                                count,
                                self._rail_slip_patience,
                            )
                        )
                else:
                    self.rail_slip_counters[rail_joint] = 0
            else:
                self.node.get_logger().warn(
                    "Rail joint %s state not available for monitoring" % rail_joint
                )
        
        return all_rails_ok
    
    def lock_rails_with_max_effort(self) -> None:
        if not self.rails_enabled:
            return
        try:
            from rclpy.utilities import ok as rclpy_ok

            if not rclpy_ok():
                return
        except Exception:
            return

        trajectory_msg = JointTrajectory()
        point = JointTrajectoryPoint()
        
        for leg_num in (1, 2, 3, 4):
            rail_joint = get_rail_joint_name(leg_num)
            trajectory_msg.joint_names.append(rail_joint)
            measured_pos = None
            if rail_joint in self.current_joint_states:
                measured_pos = self.current_joint_states[rail_joint].get('position', None)
            if measured_pos is None:
                measured_pos = self.commanded_rail_targets.get(
                    rail_joint,
                    self.last_rail_targets.get(rail_joint, 0.0),
                )
            point.positions.append(float(measured_pos))

        point.time_from_start = Duration(
            sec=int(self._rail_hold_time_from_start_sec),
            nanosec=int((self._rail_hold_time_from_start_sec % 1.0) * 1e9),
        )
        
        trajectory_msg.points.append(point)
        try:
            self.rail_trajectory_pub.publish(trajectory_msg)
        except Exception:
            return
        
        self.node.get_logger().info("Rails hold command published (position hold at measured state).")
    
    def _record_joint_command(self, joint_name: str, position: float) -> None:
        if joint_name not in self.joint_command_history:
            self.joint_command_history[joint_name] = []
        
        history_len = 10
        self.joint_command_history[joint_name].append(position)
        if len(self.joint_command_history[joint_name]) > history_len:
            self.joint_command_history[joint_name].pop(0)
    
    def detect_stuck_joints(self) -> Dict[str, bool]:
        stuck_joints = {}
        
        for joint_name in self.joint_command_history:
            if len(self.joint_command_history[joint_name]) < 3:
                continue
            
            recent_command = self.joint_command_history[joint_name][-1]
            
            if joint_name not in self.current_joint_states:
                continue
            
            actual_position = self.current_joint_states[joint_name]['position']
            
            position_error = abs(recent_command - actual_position)
            
            if position_error > self._stuck_position_error_threshold:
                if joint_name not in self.joint_stuck_count:
                    self.joint_stuck_count[joint_name] = 0
                self.joint_stuck_count[joint_name] += 1
                
                if self.joint_stuck_count[joint_name] >= self._stuck_detection_consecutive_count:
                    stuck_joints[joint_name] = True
                    
                    if self.joint_stuck_count[joint_name] == self._stuck_detection_consecutive_count:
                        self.node.get_logger().error(
                            f"Joint stuck: {joint_name} cmd={recent_command:.6f} "
                            f"actual={actual_position:.6f} err={position_error:.6f} "
                            f"(threshold={self._stuck_position_error_threshold:.6f}, "
                            f"count={self.joint_stuck_count[joint_name]})"
                        )
            else:
                if joint_name in self.joint_stuck_count:
                    self.joint_stuck_count[joint_name] = 0
                stuck_joints[joint_name] = False
        
        return stuck_joints
    
    def handle_stuck_joint(self, joint_name: str) -> None:
        self.node.get_logger().error(
            f"ALARM: Joint {joint_name} stuck. Consider switching to safety posture."
        )
    
    def check_connection(self) -> bool:
        time_since_last_update = time.monotonic() - self.last_joint_state_wall_time
        
        if time_since_last_update > self._connection_timeout_sec:
            if self.is_connected:
                self.is_connected = False
                self.node.get_logger().error(
                    f"Joint state timeout: no data for {time_since_last_update:.3f}s "
                    f"(limit={self._connection_timeout_sec:.3f}s)."
                )
            return False
        
        return True
    
    def attempt_reconnect(self) -> bool:
        if self.reconnect_attempts >= self._max_reconnect_attempts:
            self.node.get_logger().error(
                f"Reconnect aborted after {self._max_reconnect_attempts} attempts."
            )
            return False
        
        self.reconnect_attempts += 1
        self.node.get_logger().warn(
            f"Attempting reconnect check ({self.reconnect_attempts}/{self._max_reconnect_attempts})."
        )
        if self.check_connection():
            self.node.get_logger().info("Reconnected.")
            return True
        
        return False
