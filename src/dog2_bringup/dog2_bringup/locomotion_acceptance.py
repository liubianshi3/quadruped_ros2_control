#!/usr/bin/env python3
"""Ground-truth, headless locomotion acceptance for Dog2 LAV1."""

from __future__ import annotations

import csv
import math
import os
import platform
import tempfile
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, Optional, Tuple

import pinocchio as pin
import rclpy
from dog2_interfaces.msg import ContactPhase
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rcl_interfaces.srv import GetParameters
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener

from dog2_bringup.acceptance_metrics import (
    FootMetricsAccumulator,
    LEG_ORDER,
    ScalarStats,
    atomic_write_json,
    body_velocity_from_world,
    level_from_quaternion,
    parse_joint_limits,
    route_coordinates,
    transform_point,
    wrap_angle,
    yaw_from_quaternion,
)


REVOLUTE_JOINT_NAMES = tuple(
    f"{leg}_{joint}_joint"
    for leg in LEG_ORDER
    for joint in ("coxa", "femur", "tibia")
)
RAIL_JOINT_NAMES = tuple(f"{leg}_rail_joint" for leg in LEG_ORDER)


class LocomotionAcceptanceNode(Node):
    """Execute and score the LAV1 stand/out-and-back/turn protocol."""

    PARAMETER_DEFAULTS = {
        "protocol_id": "dog2_lav1_flat",
        "trial_id": "trial_001",
        "model_variant": "symmetric",
        "controller_mode": "effort_research",
        "odom_topic": "/odom",
        "dynamic_pose_topic": "/dog2/dynamic_pose_tf",
        "joint_state_topic": "/joint_states",
        "contact_phase_topic": "/dog2/gait/contact_phase",
        "effort_command_topic": "/effort_controller/commands",
        "cmd_vel_topic": "/cmd_vel",
        "foot_world_frame": "odom",
        "allow_tf_foot_fallback": True,
        "foot_contact_topic_lf": "/dog2/foot_contact/lf",
        "foot_contact_topic_lh": "/dog2/foot_contact/lh",
        "foot_contact_topic_rh": "/dog2/foot_contact/rh",
        "foot_contact_topic_rf": "/dog2/foot_contact/rf",
        "base_contact_topic": "/dog2/contact/base",
        "tibia_contact_topic_lf": "/dog2/contact/lf_tibia",
        "tibia_contact_topic_lh": "/dog2/contact/lh_tibia",
        "tibia_contact_topic_rh": "/dog2/contact/rh_tibia",
        "tibia_contact_topic_rf": "/dog2/contact/rf_tibia",
        "robot_description_node": "robot_state_publisher",
        "poll_period_sec": 0.05,
        "freshness_timeout_sec": 1.0,
        "contact_event_timeout_sec": 0.10,
        "wait_ready_timeout_sec": 60.0,
        "wait_settle_timeout_sec": 45.0,
        "total_timeout_sec": 180.0,
        "body_length_m": 0.342,
        "body_width_m": 0.160,
        "robot_mass_kg": 12.0028,
        "route_length_body_lengths": 3.0,
        "corridor_half_width_body_widths": 0.5,
        "return_tolerance_body_lengths": 0.25,
        "turn_translation_limit_body_lengths": 0.5,
        "forward_command_x": -0.05,
        "turn_command_z": 0.15,
        "turn_target_yaw_rad": math.pi / 2.0,
        "motion_timeout_factor": 2.5,
        "settle_duration_sec": 2.0,
        "stand_duration_sec": 5.0,
        "stop_duration_sec": 3.0,
        "settle_max_linear_speed_mps": 0.05,
        "settle_max_yaw_rate_radps": 0.10,
        "illegal_contact_force_n": 1.0,
        "max_tilt_rad": 0.55,
        "min_up_z": math.cos(0.55),
        "body_height_min_m": 0.12,
        "body_height_max_m": 0.40,
        "max_rail_lock_error_m": 0.005,
        "enforce_tibia_contact": True,
        "ground_z_m": 0.0,
        "foot_radius_m": 0.012,
        "result_json": "/tmp/dog2_locomotion_acceptance/trial_001.json",
        "samples_csv": "/tmp/dog2_locomotion_acceptance/trial_001_samples.csv",
        "junit_xml": "/tmp/dog2_locomotion_acceptance/trial_001.junit.xml",
    }

    SCORED_STAGES = {
        "STAND",
        "OUTBOUND",
        "OUTBOUND_STOP",
        "RETURN",
        "RETURN_STOP",
        "TURN",
        "FINAL_STOP",
    }

    def __init__(self) -> None:
        super().__init__("dog2_locomotion_acceptance")
        for name, default in self.PARAMETER_DEFAULTS.items():
            self.declare_parameter(name, default)

        self._params = {
            name: self.get_parameter(name).value for name in self.PARAMETER_DEFAULTS
        }
        self._start_wall_sec = time.monotonic()
        self._ready_deadline = self._start_wall_sec + self._float("wait_ready_timeout_sec")
        self._total_deadline = self._start_wall_sec + self._float("total_timeout_sec")
        self._stage = "WAIT_READY"
        self._stage_start_sec = self._start_wall_sec
        self._stage_deadline_sec: Optional[float] = self._ready_deadline
        self._stage_records: list[dict] = []
        self._stable_since: Optional[float] = None
        self._done = False
        self._score_start_sec: Optional[float] = None
        self._failure: Optional[dict] = None

        self._last_stream_sec: Dict[str, Optional[float]] = {
            "odom": None,
            "dynamic_pose": None,
            "foot_world_pose": None,
            "joint_state": None,
            "contact_phase": None,
            "effort_command": None,
            "base_contact": None,
        }
        for leg in LEG_ORDER:
            self._last_stream_sec[f"foot_contact_{leg}"] = None
            self._last_stream_sec[f"tibia_contact_{leg}"] = None
        self._contact_topics = {
            "base_contact": str(self._params["base_contact_topic"]),
            **{
                f"foot_contact_{leg}": str(
                    self._params[f"foot_contact_topic_{leg}"]
                )
                for leg in LEG_ORDER
            },
            **{
                f"tibia_contact_{leg}": str(
                    self._params[f"tibia_contact_topic_{leg}"]
                )
                for leg in LEG_ORDER
            },
        }

        self._latest_pose: Optional[dict] = None
        self._last_odom_stamp_sec: Optional[float] = None
        self._last_base_xy: Optional[Tuple[float, float]] = None
        self._raw_yaw_prev: Optional[float] = None
        self._unwrapped_yaw = 0.0
        self._joint_positions: Dict[str, float] = {}
        self._joint_velocities: Dict[str, float] = {}
        self._effort_commands: Dict[str, float] = {}
        self._joint_limits = {}
        self._urdf_request = None
        self._kinematic_model = None
        self._kinematic_data = None

        self._foot_contact_force = {leg: 0.0 for leg in LEG_ORDER}
        self._tibia_contact_force = {leg: 0.0 for leg in LEG_ORDER}
        self._base_contact_force = 0.0
        self._planned_stance = {leg: True for leg in LEG_ORDER}
        self._foot_positions: Dict[str, Tuple[float, float, float]] = {}
        self._dynamic_pose_frame_ids: Dict[str, str] = {}
        self._foot_position_source = "unavailable"
        self._last_tf_error = ""
        self._last_tf_error_log_sec = 0.0

        self._command_vx = 0.0
        self._command_wz = 0.0
        self._route_origin: Optional[Tuple[float, float]] = None
        self._route_axis: Optional[Tuple[float, float]] = None
        self._route_distance_m = (
            self._float("body_length_m") * self._float("route_length_body_lengths")
        )
        self._corridor_half_width_m = (
            self._float("body_width_m")
            * self._float("corridor_half_width_body_widths")
        )
        self._return_tolerance_m = (
            self._float("body_length_m")
            * self._float("return_tolerance_body_lengths")
        )
        self._turn_translation_limit_m = (
            self._float("body_length_m")
            * self._float("turn_translation_limit_body_lengths")
        )
        self._turn_origin: Optional[Tuple[float, float]] = None
        self._turn_start_yaw = 0.0

        self._height_stats = ScalarStats()
        self._tilt_stats = ScalarStats()
        self._up_z_stats = ScalarStats()
        self._vertical_velocity_stats = ScalarStats()
        self._linear_velocity_error_stats = ScalarStats()
        self._lateral_velocity_error_stats = ScalarStats()
        self._yaw_rate_error_stats = ScalarStats()
        self._lateral_error_abs_stats = ScalarStats()
        self._projection_stats = ScalarStats()
        self._base_path_length_m = 0.0
        self._commanded_energy_j = 0.0
        self._max_rail_error_m = 0.0
        self._max_base_contact_force_n = 0.0
        self._max_tibia_contact_force_n = 0.0
        self._min_joint_position_margin = math.inf
        self._max_joint_velocity_ratio = 0.0
        self._max_joint_effort_ratio = 0.0
        self._samples: list[dict] = []
        self._last_sample_wall_sec = 0.0
        self._foot_metrics = FootMetricsAccumulator(
            ground_z_m=self._float("ground_z_m"),
            foot_radius_m=self._float("foot_radius_m"),
        )
        self._tf_buffer = Buffer(cache_time=Duration(seconds=5.0), node=self)
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=False
        )

        description_node = str(self._params["robot_description_node"])
        description_service = f"/{description_node.strip('/')}/get_parameters"
        self._description_client = self.create_client(
            GetParameters, description_service
        )

        self._cmd_pub = self.create_publisher(
            Twist, str(self._params["cmd_vel_topic"]), 10
        )
        self._subscriptions = [
            self.create_subscription(
                Odometry,
                str(self._params["odom_topic"]),
                self._on_odom,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                TFMessage,
                str(self._params["dynamic_pose_topic"]),
                self._on_dynamic_pose,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                JointState,
                str(self._params["joint_state_topic"]),
                self._on_joint_state,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                ContactPhase,
                str(self._params["contact_phase_topic"]),
                self._on_contact_phase,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                Float64MultiArray,
                str(self._params["effort_command_topic"]),
                self._on_effort_command,
                qos_profile_sensor_data,
            ),
        ]
        for leg in LEG_ORDER:
            self._subscriptions.append(
                self.create_subscription(
                    Contacts,
                    str(self._params[f"foot_contact_topic_{leg}"]),
                    lambda msg, leg_name=leg: self._on_contact(
                        "foot", leg_name, msg
                    ),
                    qos_profile_sensor_data,
                )
            )
            self._subscriptions.append(
                self.create_subscription(
                    Contacts,
                    str(self._params[f"tibia_contact_topic_{leg}"]),
                    lambda msg, leg_name=leg: self._on_contact(
                        "tibia", leg_name, msg
                    ),
                    qos_profile_sensor_data,
                )
            )
        self._subscriptions.append(
            self.create_subscription(
                Contacts,
                str(self._params["base_contact_topic"]),
                lambda msg: self._on_contact("base", "base", msg),
                qos_profile_sensor_data,
            )
        )

        self.create_timer(self._float("poll_period_sec"), self._poll)
        self.get_logger().info(
            "LAV1 waiting for Gazebo ground truth; "
            f"trial={self._params['trial_id']} route={self._route_distance_m:.3f}m"
        )

    def _float(self, name: str) -> float:
        return float(self._params[name])

    @staticmethod
    def _now_sec() -> float:
        return time.monotonic()

    @staticmethod
    def _message_stamp_sec(sec: int, nanosec: int) -> float:
        value = float(sec) + float(nanosec) * 1e-9
        return value if value > 0.0 else time.monotonic()

    def _age(self, name: str) -> float:
        stamp = self._last_stream_sec.get(name)
        if stamp is None:
            return math.inf
        return max(0.0, self._now_sec() - stamp)

    def _is_fresh(self, name: str) -> bool:
        return self._age(name) <= self._float("freshness_timeout_sec")

    def _event_force(self, stream_name: str, recorded_force: float) -> float:
        if self._age(stream_name) > self._float("contact_event_timeout_sec"):
            return 0.0
        return recorded_force

    def _current_foot_force(self, leg: str) -> float:
        return self._event_force(
            f"foot_contact_{leg}", self._foot_contact_force[leg]
        )

    def _current_tibia_force(self, leg: str) -> float:
        return self._event_force(
            f"tibia_contact_{leg}", self._tibia_contact_force[leg]
        )

    def _current_base_force(self) -> float:
        return self._event_force("base_contact", self._base_contact_force)

    @staticmethod
    def _contact_force_n(msg: Contacts, fallback_force_n: float) -> float:
        maximum = 0.0
        for contact in msg.contacts:
            for wrench in contact.wrenches:
                for body_wrench in (
                    wrench.body_1_wrench,
                    wrench.body_2_wrench,
                ):
                    force = body_wrench.force
                    maximum = max(
                        maximum,
                        math.sqrt(
                            float(force.x) ** 2
                            + float(force.y) ** 2
                            + float(force.z) ** 2
                        ),
                    )
        if msg.contacts and maximum <= 0.0:
            maximum = fallback_force_n
        return maximum

    def _on_contact(self, kind: str, leg: str, msg: Contacts) -> None:
        now = self._now_sec()
        force = self._contact_force_n(
            msg, self._float("illegal_contact_force_n")
        )
        if kind == "base":
            self._base_contact_force = force
            self._last_stream_sec["base_contact"] = now
            self._max_base_contact_force_n = max(
                self._max_base_contact_force_n, force
            )
        elif kind == "tibia":
            self._tibia_contact_force[leg] = force
            self._last_stream_sec[f"tibia_contact_{leg}"] = now
            self._max_tibia_contact_force_n = max(
                self._max_tibia_contact_force_n, force
            )
        else:
            self._foot_contact_force[leg] = force
            self._last_stream_sec[f"foot_contact_{leg}"] = now

    def _on_contact_phase(self, msg: ContactPhase) -> None:
        mapping = {}
        if len(msg.leg_names) == len(msg.phase):
            mapping = {
                str(name).lower(): int(value)
                for name, value in zip(msg.leg_names, msg.phase)
            }
        if not all(leg in mapping for leg in LEG_ORDER):
            mapping = {
                leg: int(msg.phase[index])
                for index, leg in enumerate(LEG_ORDER)
                if index < len(msg.phase)
            }
        if all(leg in mapping for leg in LEG_ORDER):
            self._planned_stance = {
                leg: mapping[leg] == int(ContactPhase.STANCE) for leg in LEG_ORDER
            }
            self._last_stream_sec["contact_phase"] = self._now_sec()

    def _on_joint_state(self, msg: JointState) -> None:
        for index, name in enumerate(msg.name):
            if index < len(msg.position):
                self._joint_positions[str(name)] = float(msg.position[index])
            if index < len(msg.velocity):
                self._joint_velocities[str(name)] = float(msg.velocity[index])
        self._last_stream_sec["joint_state"] = self._now_sec()

    def _on_effort_command(self, msg: Float64MultiArray) -> None:
        if len(msg.data) >= len(REVOLUTE_JOINT_NAMES):
            self._effort_commands = {
                name: float(msg.data[index])
                for index, name in enumerate(REVOLUTE_JOINT_NAMES)
            }
            self._last_stream_sec["effort_command"] = self._now_sec()

    @staticmethod
    def _frame_matches(child_frame_id: str, frame: str) -> bool:
        child = child_frame_id.lower()
        frame = frame.lower()
        return (
            child == frame
            or child.endswith(f"/{frame}")
            or child.endswith(f"::{frame}")
        )

    def _on_dynamic_pose(self, msg: TFMessage) -> None:
        now = self._now_sec()
        positions: Dict[str, Tuple[float, float, float]] = {}
        stamp_sec = now
        frame_ids: Dict[str, str] = {}
        for leg in LEG_ORDER:
            frame = f"{leg}_foot_link"
            transform = next(
                (
                    candidate
                    for candidate in msg.transforms
                    if self._frame_matches(candidate.child_frame_id, frame)
                ),
                None,
            )
            if transform is None:
                drive_frame = f"{leg}_tibia_drive_frame"
                drive_transform = next(
                    (
                        candidate
                        for candidate in msg.transforms
                        if self._frame_matches(candidate.child_frame_id, drive_frame)
                    ),
                    None,
                )
                if drive_transform is None:
                    continue
                try:
                    fixed_transform = self._tf_buffer.lookup_transform(
                        drive_frame,
                        frame,
                        Time(),
                    )
                except TransformException:
                    continue

                drive_translation = drive_transform.transform.translation
                drive_rotation = drive_transform.transform.rotation
                fixed_translation = fixed_transform.transform.translation
                positions[leg] = transform_point(
                    (
                        float(drive_translation.x),
                        float(drive_translation.y),
                        float(drive_translation.z),
                    ),
                    (
                        float(drive_rotation.x),
                        float(drive_rotation.y),
                        float(drive_rotation.z),
                        float(drive_rotation.w),
                    ),
                    (
                        float(fixed_translation.x),
                        float(fixed_translation.y),
                        float(fixed_translation.z),
                    ),
                )
                frame_ids[leg] = (
                    f"{drive_transform.header.frame_id}"
                    f"->{drive_transform.child_frame_id}"
                    f"->{frame}"
                )
                stamp_sec = self._message_stamp_sec(
                    drive_transform.header.stamp.sec,
                    drive_transform.header.stamp.nanosec,
                )
                continue
            translation = transform.transform.translation
            positions[leg] = (
                float(translation.x),
                float(translation.y),
                float(translation.z),
            )
            frame_ids[leg] = (
                f"{transform.header.frame_id}->{transform.child_frame_id}"
            )
            stamp_sec = self._message_stamp_sec(
                transform.header.stamp.sec,
                transform.header.stamp.nanosec,
            )

        if len(positions) == len(LEG_ORDER):
            self._last_stream_sec["dynamic_pose"] = now
            self._accept_foot_positions(
                positions,
                frame_ids,
                stamp_sec,
                source="gazebo_dynamic_pose",
            )

    def _accept_foot_positions(
        self,
        positions: Dict[str, Tuple[float, float, float]],
        frame_ids: Dict[str, str],
        stamp_sec: float,
        *,
        source: str,
    ) -> None:
        self._foot_positions = positions
        self._dynamic_pose_frame_ids = frame_ids
        self._foot_position_source = source
        self._last_stream_sec["foot_world_pose"] = self._now_sec()
        if self._stage in self.SCORED_STAGES:
            threshold = self._float("illegal_contact_force_n")
            actual = {
                leg: self._current_foot_force(leg) >= threshold
                for leg in LEG_ORDER
            }
            self._foot_metrics.update(
                stamp_sec,
                positions,
                actual,
                self._planned_stance,
            )

    def _update_foot_positions_from_tf(self) -> None:
        if not bool(self._params["allow_tf_foot_fallback"]):
            return
        if self._is_fresh("dynamic_pose"):
            return
        target_frame = str(self._params["foot_world_frame"])
        positions: Dict[str, Tuple[float, float, float]] = {}
        frame_ids: Dict[str, str] = {}
        for leg in LEG_ORDER:
            source_frame = f"{leg}_foot_link"
            try:
                transform = self._tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    Time(),
                )
            except TransformException as exc:
                kinematic_positions = self._kinematic_foot_positions(target_frame)
                if len(kinematic_positions) == len(LEG_ORDER):
                    self._accept_foot_positions(
                        kinematic_positions,
                        {
                            leg_name: f"{target_frame}->base_link+pinocchio->{leg_name}_foot_link"
                            for leg_name in LEG_ORDER
                        },
                        self._now_sec(),
                        source="gazebo_odom+joint_state_pinocchio",
                    )
                    return
                self._log_tf_wait(target_frame, source_frame, exc)
                return
            translation = transform.transform.translation
            positions[leg] = (
                float(translation.x),
                float(translation.y),
                float(translation.z),
            )
            frame_ids[leg] = (
                f"{transform.header.frame_id}->{transform.child_frame_id}"
            )
        self._accept_foot_positions(
            positions,
            frame_ids,
            self._now_sec(),
            source="tf_kinematic_fallback",
        )

    def _kinematic_foot_positions(
        self, target_frame: str
    ) -> Dict[str, Tuple[float, float, float]]:
        pose = self._latest_pose
        model = self._kinematic_model
        data = self._kinematic_data
        if pose is None or model is None or data is None:
            return {}
        if str(pose.get("frame_id", "")).lstrip("/") != target_frame.lstrip("/"):
            return {}

        configuration = pin.neutral(model)
        for joint_id in range(1, model.njoints):
            joint = model.joints[joint_id]
            name = str(model.names[joint_id])
            if joint.nq != 1 or name not in self._joint_positions:
                return {}
            configuration[joint.idx_q] = self._joint_positions[name]

        pin.forwardKinematics(model, data, configuration)
        pin.updateFramePlacements(model, data)
        positions: Dict[str, Tuple[float, float, float]] = {}
        for leg in LEG_ORDER:
            frame_name = f"{leg}_foot_link"
            frame_id = model.getFrameId(frame_name)
            if frame_id >= model.nframes:
                return {}
            translation = data.oMf[frame_id].translation
            positions[leg] = transform_point(
                (pose["x"], pose["y"], pose["z"]),
                (pose["qx"], pose["qy"], pose["qz"], pose["qw"]),
                (
                    float(translation[0]),
                    float(translation[1]),
                    float(translation[2]),
                ),
            )
        return positions

    def _log_tf_wait(
        self, target_frame: str, source_frame: str, exc: TransformException
    ) -> None:
        self._last_tf_error = str(exc)
        now = self._now_sec()
        if now - self._last_tf_error_log_sec < 5.0:
            return
        self._last_tf_error_log_sec = now
        self.get_logger().warn(
            f"Waiting for TF {target_frame}->{source_frame}: {exc}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        now = self._now_sec()
        pose = msg.pose.pose
        twist = msg.twist.twist
        q = pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        roll_like, pitch_like, tilt, up_z = level_from_quaternion(
            q.x, q.y, q.z, q.w
        )
        vx_body, vy_body = body_velocity_from_world(
            twist.linear.x,
            twist.linear.y,
            yaw,
        )
        if self._raw_yaw_prev is None:
            self._unwrapped_yaw = yaw
        else:
            self._unwrapped_yaw += wrap_angle(yaw - self._raw_yaw_prev)
        self._raw_yaw_prev = yaw

        projection = 0.0
        lateral = 0.0
        if self._route_origin is not None and self._route_axis is not None:
            projection, lateral = route_coordinates(
                pose.position.x,
                pose.position.y,
                self._route_origin[0],
                self._route_origin[1],
                self._route_axis[0],
                self._route_axis[1],
            )

        stamp_sec = self._message_stamp_sec(
            msg.header.stamp.sec, msg.header.stamp.nanosec
        )
        dt = 0.0
        if self._last_odom_stamp_sec is not None:
            candidate = stamp_sec - self._last_odom_stamp_sec
            if 0.0 < candidate <= 0.25:
                dt = candidate
        self._last_odom_stamp_sec = stamp_sec

        self._latest_pose = {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
            "qx": float(q.x),
            "qy": float(q.y),
            "qz": float(q.z),
            "qw": float(q.w),
            "frame_id": str(msg.header.frame_id),
            "child_frame_id": str(msg.child_frame_id),
            "yaw": yaw,
            "unwrapped_yaw": self._unwrapped_yaw,
            "roll_like": roll_like,
            "pitch_like": pitch_like,
            "tilt": tilt,
            "up_z": up_z,
            "vx_body": vx_body,
            "vy_body": vy_body,
            "vz": float(twist.linear.z),
            "wz": float(twist.angular.z),
            "projection": projection,
            "lateral": lateral,
        }
        self._last_stream_sec["odom"] = now

        if self._stage not in self.SCORED_STAGES:
            self._last_base_xy = (pose.position.x, pose.position.y)
            return

        self._height_stats.update(pose.position.z)
        self._tilt_stats.update(tilt)
        self._up_z_stats.update(up_z)
        self._vertical_velocity_stats.update(twist.linear.z)
        self._linear_velocity_error_stats.update(vx_body - self._command_vx)
        self._lateral_velocity_error_stats.update(vy_body)
        self._yaw_rate_error_stats.update(twist.angular.z - self._command_wz)
        self._lateral_error_abs_stats.update(abs(lateral))
        self._projection_stats.update(projection)

        current_xy = (float(pose.position.x), float(pose.position.y))
        if self._last_base_xy is not None:
            self._base_path_length_m += math.hypot(
                current_xy[0] - self._last_base_xy[0],
                current_xy[1] - self._last_base_xy[1],
            )
        self._last_base_xy = current_xy

        if dt > 0.0 and self._effort_commands:
            power = 0.0
            for name, effort in self._effort_commands.items():
                power += abs(effort * self._joint_velocities.get(name, 0.0))
            self._commanded_energy_j += power * dt

        if now - self._last_sample_wall_sec >= self._float("poll_period_sec"):
            self._last_sample_wall_sec = now
            self._samples.append(self._sample_row(now))

    def _sample_row(self, now: float) -> dict:
        pose = self._latest_pose or {}
        row = {
            "time_sec": (
                0.0
                if self._score_start_sec is None
                else max(0.0, now - self._score_start_sec)
            ),
            "stage": self._stage,
            "cmd_vx_mps": self._command_vx,
            "cmd_wz_radps": self._command_wz,
            "x_m": pose.get("x"),
            "y_m": pose.get("y"),
            "z_m": pose.get("z"),
            "yaw_rad": pose.get("yaw"),
            "unwrapped_yaw_rad": pose.get("unwrapped_yaw"),
            "tilt_rad": pose.get("tilt"),
            "up_z": pose.get("up_z"),
            "vx_body_mps": pose.get("vx_body"),
            "vy_body_mps": pose.get("vy_body"),
            "vz_mps": pose.get("vz"),
            "wz_radps": pose.get("wz"),
            "route_projection_m": pose.get("projection"),
            "lateral_error_m": pose.get("lateral"),
            "rail_error_max_m": self._current_rail_error(),
            "base_contact_force_n": self._current_base_force(),
            "tibia_contact_force_max_n": max(
                self._current_tibia_force(leg) for leg in LEG_ORDER
            ),
        }
        for leg in LEG_ORDER:
            row[f"{leg}_foot_contact"] = int(
                self._current_foot_force(leg)
                >= self._float("illegal_contact_force_n")
            )
            row[f"{leg}_planned_stance"] = int(self._planned_stance[leg])
        return row

    def _request_robot_description(self) -> None:
        if self._joint_limits or self._urdf_request is not None:
            return
        if not self._description_client.service_is_ready():
            return
        request = GetParameters.Request()
        request.names = ["robot_description"]
        self._urdf_request = self._description_client.call_async(request)

    def _collect_robot_description(self) -> None:
        if self._joint_limits:
            return
        if self._urdf_request is None or not self._urdf_request.done():
            return
        try:
            response = self._urdf_request.result()
            description = response.values[0].string_value
            joint_limits = parse_joint_limits(description)
            kinematic_model = pin.buildModelFromXML(description)
            expected = set(RAIL_JOINT_NAMES + REVOLUTE_JOINT_NAMES)
            missing = sorted(expected - set(joint_limits))
            if missing:
                raise ValueError(f"URDF missing movable joints: {missing}")
            self._joint_limits = joint_limits
            self._kinematic_model = kinematic_model
            self._kinematic_data = kinematic_model.createData()
            self.get_logger().info(
                f"Loaded {len(self._joint_limits)} movable-joint hard limits"
            )
            self._urdf_request = None
        except Exception as exc:
            self._urdf_request = None
            self.get_logger().warn(f"Could not read robot_description yet: {exc}")

    def _required_streams_missing(self) -> list[str]:
        names = [
            "odom",
            "foot_world_pose",
            "joint_state",
            "contact_phase",
            "effort_command",
        ]
        missing = [name for name in names if not self._is_fresh(name)]
        missing.extend(
            f"{name}_publisher"
            for name, topic in self._contact_topics.items()
            if self.count_publishers(topic) < 1
        )
        if not self._joint_limits:
            missing.append("robot_description")
        if set(self._foot_positions) != set(LEG_ORDER):
            missing.append("world_foot_positions")
        return missing

    def _publish_command(self, vx: float, wz: float) -> None:
        self._command_vx = float(vx)
        self._command_wz = float(wz)
        msg = Twist()
        msg.linear.x = self._command_vx
        msg.angular.z = self._command_wz
        self._cmd_pub.publish(msg)

    def _begin_stage(
        self, stage: str, *, timeout_sec: Optional[float] = None
    ) -> None:
        now = self._now_sec()
        self._close_stage(now)
        self._stage = stage
        self._stage_start_sec = now
        self._stage_deadline_sec = (
            None if timeout_sec is None else now + max(0.1, timeout_sec)
        )
        self._stable_since = None
        self.get_logger().info(f"LAV1 stage={stage}")

    def _close_stage(self, now: Optional[float] = None) -> None:
        if not self._stage:
            return
        now = self._now_sec() if now is None else now
        if self._stage_records and self._stage_records[-1].get("stage") == self._stage:
            return
        self._stage_records.append(
            {
                "stage": self._stage,
                "start_wall_sec": self._stage_start_sec - self._start_wall_sec,
                "end_wall_sec": now - self._start_wall_sec,
                "duration_sec": max(0.0, now - self._stage_start_sec),
            }
        )

    def _current_rail_error(self) -> float:
        if not all(name in self._joint_positions for name in RAIL_JOINT_NAMES):
            return math.inf
        return max(abs(self._joint_positions[name]) for name in RAIL_JOINT_NAMES)

    def _is_stable(self) -> bool:
        pose = self._latest_pose
        if pose is None:
            return False
        speed = math.hypot(pose["vx_body"], pose["vy_body"])
        no_illegal_contact = (
            self._current_base_force() < self._float("illegal_contact_force_n")
            and (
                not bool(self._params["enforce_tibia_contact"])
                or max(self._current_tibia_force(leg) for leg in LEG_ORDER)
                < self._float("illegal_contact_force_n")
            )
        )
        return (
            no_illegal_contact
            and speed <= self._float("settle_max_linear_speed_mps")
            and abs(pose["wz"]) <= self._float("settle_max_yaw_rate_radps")
            and pose["tilt"] <= self._float("max_tilt_rad")
            and pose["up_z"] >= self._float("min_up_z")
            and self._float("body_height_min_m")
            <= pose["z"]
            <= self._float("body_height_max_m")
            and self._current_rail_error()
            <= self._float("max_rail_lock_error_m")
        )

    def _stable_for(self, duration_sec: float) -> bool:
        now = self._now_sec()
        if self._is_stable():
            if self._stable_since is None:
                self._stable_since = now
            return now - self._stable_since >= duration_sec
        self._stable_since = None
        return False

    def _fail(
        self,
        code: str,
        message: str,
        *,
        infrastructure: bool = False,
        measured=None,
        limit=None,
        provenance: str = "PROJECT_SAFETY",
    ) -> None:
        if self._done:
            return
        self._failure = {
            "code": code,
            "message": message,
            "stage": self._stage,
            "time_sec": self._now_sec() - self._start_wall_sec,
            "measured": measured,
            "limit": limit,
            "provenance": provenance,
        }
        status = "FAIL_INFRASTRUCTURE" if infrastructure else "FAIL_LOCOMOTION"
        self._finish(False, status, message)

    def _check_hard_gates(self) -> bool:
        missing = self._required_streams_missing()
        if missing:
            self._fail(
                "STALE_GROUND_TRUTH",
                f"required streams stale or missing: {missing}",
                infrastructure=True,
                measured=missing,
                limit=self._float("freshness_timeout_sec"),
                provenance="TEST_INFRASTRUCTURE",
            )
            return False

        pose = self._latest_pose
        assert pose is not None
        finite_values = [
            pose[key]
            for key in (
                "x",
                "y",
                "z",
                "yaw",
                "tilt",
                "up_z",
                "vx_body",
                "vy_body",
                "wz",
            )
        ]
        if not all(math.isfinite(value) for value in finite_values):
            self._fail(
                "NONFINITE_STATE",
                "non-finite Gazebo ground-truth state",
                measured=finite_values,
                provenance="PHYSICAL_HARD",
            )
            return False

        threshold = self._float("illegal_contact_force_n")
        base_force = self._current_base_force()
        if base_force >= threshold:
            self._fail(
                "BASE_CONTACT",
                "base_link made illegal contact",
                measured=base_force,
                limit=threshold,
                provenance="REFERENCE_HARD",
            )
            return False
        if bool(self._params["enforce_tibia_contact"]):
            leg, force = max(
                (
                    (leg_name, self._current_tibia_force(leg_name))
                    for leg_name in LEG_ORDER
                ),
                key=lambda item: item[1],
            )
            if force >= threshold:
                self._fail(
                    "TIBIA_CONTACT",
                    f"{leg}_tibia_link made illegal contact",
                    measured=force,
                    limit=threshold,
                    provenance="PROJECT_SAFETY",
                )
                return False
        if pose["tilt"] > self._float("max_tilt_rad"):
            self._fail(
                "TILT_LIMIT",
                "body tilt exceeded safety limit",
                measured=pose["tilt"],
                limit=self._float("max_tilt_rad"),
            )
            return False
        if pose["up_z"] < self._float("min_up_z"):
            self._fail(
                "UP_Z_LIMIT",
                "body +Z no longer points sufficiently upward",
                measured=pose["up_z"],
                limit=self._float("min_up_z"),
            )
            return False
        if not (
            self._float("body_height_min_m")
            <= pose["z"]
            <= self._float("body_height_max_m")
        ):
            self._fail(
                "BODY_HEIGHT_LIMIT",
                "body height left the accepted band",
                measured=pose["z"],
                limit=[
                    self._float("body_height_min_m"),
                    self._float("body_height_max_m"),
                ],
            )
            return False

        rail_error = self._current_rail_error()
        self._max_rail_error_m = max(self._max_rail_error_m, rail_error)
        if rail_error > self._float("max_rail_lock_error_m"):
            self._fail(
                "RAIL_LOCK_ERROR",
                "one or more prismatic rails left the zero-lock tolerance",
                measured=rail_error,
                limit=self._float("max_rail_lock_error_m"),
                provenance="MISSION_REQUIREMENT",
            )
            return False

        for name, limit in self._joint_limits.items():
            if name not in self._joint_positions:
                continue
            position = self._joint_positions[name]
            margins = []
            if limit.lower is not None:
                margins.append(position - limit.lower)
                if position < limit.lower - 1e-6:
                    self._fail(
                        "JOINT_POSITION_LIMIT",
                        f"{name} is below its URDF hard limit",
                        measured=position,
                        limit=limit.lower,
                        provenance="PHYSICAL_HARD",
                    )
                    return False
            if limit.upper is not None:
                margins.append(limit.upper - position)
                if position > limit.upper + 1e-6:
                    self._fail(
                        "JOINT_POSITION_LIMIT",
                        f"{name} is above its URDF hard limit",
                        measured=position,
                        limit=limit.upper,
                        provenance="PHYSICAL_HARD",
                    )
                    return False
            if margins:
                self._min_joint_position_margin = min(
                    self._min_joint_position_margin, *margins
                )

            velocity = abs(self._joint_velocities.get(name, 0.0))
            if limit.velocity is not None and limit.velocity > 0.0:
                ratio = velocity / limit.velocity
                self._max_joint_velocity_ratio = max(
                    self._max_joint_velocity_ratio, ratio
                )
                if ratio > 1.0 + 1e-6:
                    self._fail(
                        "JOINT_VELOCITY_LIMIT",
                        f"{name} exceeded its URDF velocity limit",
                        measured=velocity,
                        limit=limit.velocity,
                        provenance="PHYSICAL_HARD",
                    )
                    return False

            effort = abs(self._effort_commands.get(name, 0.0))
            if name in self._effort_commands and limit.effort is not None:
                ratio = effort / max(limit.effort, 1e-12)
                self._max_joint_effort_ratio = max(
                    self._max_joint_effort_ratio, ratio
                )
                if ratio > 1.0 + 1e-6:
                    self._fail(
                        "JOINT_EFFORT_LIMIT",
                        f"{name} command exceeded its URDF effort limit",
                        measured=effort,
                        limit=limit.effort,
                        provenance="PHYSICAL_HARD",
                    )
                    return False

        if (
            self._route_origin is not None
            and abs(pose["lateral"]) > self._corridor_half_width_m
        ):
            self._fail(
                "ROUTE_CORRIDOR",
                "body left the fixed-route lateral corridor",
                measured=abs(pose["lateral"]),
                limit=self._corridor_half_width_m,
                provenance="PROJECT_SAFETY",
            )
            return False
        return True

    def _motion_timeout(self, distance_or_angle: float, speed: float) -> float:
        nominal = abs(distance_or_angle) / max(abs(speed), 1e-6)
        return max(5.0, nominal * self._float("motion_timeout_factor"))

    def _start_scored_protocol(self) -> None:
        assert self._latest_pose is not None
        pose = self._latest_pose
        command_sign = -1.0 if self._float("forward_command_x") < 0.0 else 1.0
        self._route_origin = (pose["x"], pose["y"])
        self._route_axis = (
            command_sign * math.cos(pose["yaw"]),
            command_sign * math.sin(pose["yaw"]),
        )
        self._score_start_sec = self._now_sec()
        self._last_base_xy = self._route_origin
        self._begin_stage("STAND", timeout_sec=self._float("stand_duration_sec") + 5.0)

    def _poll(self) -> None:
        if self._done:
            return
        now = self._now_sec()
        self._request_robot_description()
        self._collect_robot_description()
        self._update_foot_positions_from_tf()

        if now >= self._total_deadline:
            self._fail(
                "TOTAL_TIMEOUT",
                "LAV1 exceeded its total wall-clock timeout",
                infrastructure=True,
                limit=self._float("total_timeout_sec"),
                provenance="TEST_INFRASTRUCTURE",
            )
            return

        if self._stage == "WAIT_READY":
            self._publish_command(0.0, 0.0)
            missing = self._required_streams_missing()
            if not missing:
                self._begin_stage(
                    "WAIT_SETTLE",
                    timeout_sec=self._float("wait_settle_timeout_sec"),
                )
            elif now >= self._ready_deadline:
                self._fail(
                    "READINESS_TIMEOUT",
                    f"required ground-truth never became ready: {missing}",
                    infrastructure=True,
                    measured=missing,
                    limit=self._float("wait_ready_timeout_sec"),
                    provenance="TEST_INFRASTRUCTURE",
                )
            return

        if self._stage == "WAIT_SETTLE":
            self._publish_command(0.0, 0.0)
            missing = self._required_streams_missing()
            if missing:
                self._stable_since = None
            elif self._stable_for(self._float("settle_duration_sec")):
                self._start_scored_protocol()
                return
            if self._stage_deadline_sec is not None and now >= self._stage_deadline_sec:
                self._fail(
                    "SETTLE_TIMEOUT",
                    "robot did not reach the required stable start state",
                    measured=self._latest_pose,
                    limit=self._float("wait_settle_timeout_sec"),
                )
            return

        if not self._check_hard_gates():
            return
        assert self._latest_pose is not None
        pose = self._latest_pose
        elapsed = now - self._stage_start_sec

        if self._stage == "STAND":
            self._publish_command(0.0, 0.0)
            if elapsed >= self._float("stand_duration_sec"):
                timeout = self._motion_timeout(
                    self._route_distance_m,
                    self._float("forward_command_x"),
                )
                self._begin_stage("OUTBOUND", timeout_sec=timeout)

        elif self._stage == "OUTBOUND":
            self._publish_command(self._float("forward_command_x"), 0.0)
            if pose["projection"] >= self._route_distance_m:
                self._begin_stage(
                    "OUTBOUND_STOP",
                    timeout_sec=self._float("stop_duration_sec") + 8.0,
                )

        elif self._stage == "OUTBOUND_STOP":
            self._publish_command(0.0, 0.0)
            if (
                elapsed >= self._float("stop_duration_sec")
                and self._stable_for(1.0)
            ):
                timeout = self._motion_timeout(
                    self._route_distance_m,
                    self._float("forward_command_x"),
                )
                self._begin_stage("RETURN", timeout_sec=timeout)

        elif self._stage == "RETURN":
            self._publish_command(-self._float("forward_command_x"), 0.0)
            if pose["projection"] <= self._return_tolerance_m:
                self._begin_stage(
                    "RETURN_STOP",
                    timeout_sec=self._float("stop_duration_sec") + 8.0,
                )

        elif self._stage == "RETURN_STOP":
            self._publish_command(0.0, 0.0)
            if (
                elapsed >= self._float("stop_duration_sec")
                and self._stable_for(1.0)
            ):
                self._turn_origin = (pose["x"], pose["y"])
                self._turn_start_yaw = pose["unwrapped_yaw"]
                timeout = self._motion_timeout(
                    self._float("turn_target_yaw_rad"),
                    self._float("turn_command_z"),
                )
                self._begin_stage("TURN", timeout_sec=timeout)

        elif self._stage == "TURN":
            self._publish_command(0.0, self._float("turn_command_z"))
            assert self._turn_origin is not None
            translation = math.hypot(
                pose["x"] - self._turn_origin[0],
                pose["y"] - self._turn_origin[1],
            )
            if translation > self._turn_translation_limit_m:
                self._fail(
                    "TURN_TRANSLATION",
                    "in-place turn translated beyond its allowed radius",
                    measured=translation,
                    limit=self._turn_translation_limit_m,
                    provenance="PROJECT_SAFETY",
                )
                return
            direction = -1.0 if self._float("turn_command_z") < 0.0 else 1.0
            yaw_progress = direction * (
                pose["unwrapped_yaw"] - self._turn_start_yaw
            )
            if yaw_progress >= self._float("turn_target_yaw_rad"):
                self._begin_stage(
                    "FINAL_STOP",
                    timeout_sec=self._float("stop_duration_sec") + 8.0,
                )

        elif self._stage == "FINAL_STOP":
            self._publish_command(0.0, 0.0)
            if (
                elapsed >= self._float("stop_duration_sec")
                and self._stable_for(1.0)
            ):
                self._finish(
                    True,
                    "PASS_LOCOMOTION_BASELINE",
                    "all LAV1 hard and task gates passed",
                )
                return

        if self._stage_deadline_sec is not None and now >= self._stage_deadline_sec:
            self._fail(
                "STAGE_TIMEOUT",
                f"stage {self._stage} did not meet its completion condition",
                measured=pose,
                limit=self._stage_deadline_sec - self._stage_start_sec,
                provenance="TASK_PROTOCOL",
            )

    def _gate_summary(self) -> dict:
        return {
            "illegal_base_contact": {
                "status": (
                    "FAIL"
                    if self._failure and self._failure["code"] == "BASE_CONTACT"
                    else "PASS"
                ),
                "maximum_n": self._max_base_contact_force_n,
                "limit_n": self._float("illegal_contact_force_n"),
                "provenance": "REFERENCE_HARD",
            },
            "illegal_tibia_contact": {
                "status": (
                    "FAIL"
                    if self._failure and self._failure["code"] == "TIBIA_CONTACT"
                    else "PASS"
                ),
                "maximum_n": self._max_tibia_contact_force_n,
                "limit_n": self._float("illegal_contact_force_n"),
                "provenance": "PROJECT_SAFETY",
            },
            "tilt": {
                "maximum_rad": self._tilt_stats.maximum
                if self._tilt_stats.count
                else None,
                "limit_rad": self._float("max_tilt_rad"),
                "provenance": "PROJECT_SAFETY",
            },
            "rail_lock": {
                "maximum_error_m": self._max_rail_error_m,
                "limit_m": self._float("max_rail_lock_error_m"),
                "provenance": "MISSION_REQUIREMENT",
            },
            "joint_limits": {
                "minimum_position_margin": (
                    self._min_joint_position_margin
                    if math.isfinite(self._min_joint_position_margin)
                    else None
                ),
                "maximum_velocity_ratio": self._max_joint_velocity_ratio,
                "maximum_effort_ratio": self._max_joint_effort_ratio,
                "provenance": "PHYSICAL_HARD",
            },
            "route": {
                "target_distance_m": self._route_distance_m,
                "corridor_half_width_m": self._corridor_half_width_m,
                "return_tolerance_m": self._return_tolerance_m,
                "provenance": "TASK_PROTOCOL",
            },
        }

    def _metrics_report(self) -> dict:
        foot_metrics = self._foot_metrics.as_dict()
        travel = max(self._base_path_length_m, 1e-9)
        mass = max(self._float("robot_mass_kg"), 1e-9)
        commanded_cot = self._commanded_energy_j / (mass * 9.80665 * travel)
        return {
            "body_height_m": self._height_stats.as_dict(),
            "tilt_rad": self._tilt_stats.as_dict(),
            "up_z": self._up_z_stats.as_dict(),
            "vertical_velocity_mps": self._vertical_velocity_stats.as_dict(),
            "linear_velocity_error_mps": self._linear_velocity_error_stats.as_dict(),
            "lateral_velocity_error_mps": self._lateral_velocity_error_stats.as_dict(),
            "yaw_rate_error_radps": self._yaw_rate_error_stats.as_dict(),
            "lateral_error_abs_m": self._lateral_error_abs_stats.as_dict(),
            "normalized_lateral_error_mean": (
                None
                if self._lateral_error_abs_stats.mean is None
                else self._lateral_error_abs_stats.mean
                / max(self._float("body_width_m"), 1e-9)
            ),
            "normalized_lateral_error_max": (
                None
                if self._lateral_error_abs_stats.count == 0
                else self._lateral_error_abs_stats.maximum
                / max(self._float("body_width_m"), 1e-9)
            ),
            "route_projection_m": self._projection_stats.as_dict(),
            "base_path_length_m": self._base_path_length_m,
            "commanded_mechanical_energy_j": self._commanded_energy_j,
            "commanded_cot": commanded_cot,
            "foot": foot_metrics,
            "stance_slip_per_base_path": (
                foot_metrics["stance_slip_distance_total_m"] / travel
            ),
        }

    def _build_report(self, status: str, message: str) -> dict:
        pose = self._latest_pose
        return {
            "schema_version": 1,
            "protocol_id": str(self._params["protocol_id"]),
            "trial_id": str(self._params["trial_id"]),
            "status": status,
            "passed": status == "PASS_LOCOMOTION_BASELINE",
            "message": message,
            "failure": self._failure,
            "environment": {
                "model_variant": str(self._params["model_variant"]),
                "controller_mode": str(self._params["controller_mode"]),
                "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
                "platform": platform.platform(),
                "python": platform.python_version(),
            },
            "timing": {
                "wall_duration_sec": self._now_sec() - self._start_wall_sec,
                "score_duration_sec": (
                    None
                    if self._score_start_sec is None
                    else self._now_sec() - self._score_start_sec
                ),
            },
            "stages": self._stage_records,
            "final_pose": pose,
            "gates": self._gate_summary(),
            "metrics": self._metrics_report(),
            "data_sources": {
                "odom": str(self._params["odom_topic"]),
                "dynamic_pose": str(self._params["dynamic_pose_topic"]),
                "foot_world_position_source": self._foot_position_source,
                "joint_state": str(self._params["joint_state_topic"]),
                "contact_phase": str(self._params["contact_phase_topic"]),
                "effort_command": str(self._params["effort_command_topic"]),
                "dynamic_pose_frames": self._dynamic_pose_frame_ids,
                "last_tf_error": self._last_tf_error or None,
            },
            "threshold_provenance": {
                "REFERENCE_HARD": "Isaac Lab / legged_gym termination pattern",
                "PHYSICAL_HARD": "expanded Dog2 URDF joint limits",
                "MISSION_REQUIREMENT": "Dog2 rail lock requirement",
                "PROJECT_SAFETY": "Dog2-specific configurable safety gate",
                "REPORT_ONLY": "reported PI; no universal PASS threshold claimed",
            },
            "specification": "docs/DOG2_LOCOMOTION_ACCEPTANCE_SPEC.md",
        }

    def _write_samples(self) -> None:
        path = Path(str(self._params["samples_csv"]))
        path.parent.mkdir(parents=True, exist_ok=True)
        if self._samples:
            fieldnames = list(self._samples[0])
        else:
            fieldnames = ["time_sec", "stage"]
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            newline="",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self._samples)
            temp_name = handle.name
        os.replace(temp_name, path)

    def _write_junit(self, report: dict) -> None:
        passed = bool(report["passed"])
        infrastructure = report["status"] == "FAIL_INFRASTRUCTURE"
        suite = ET.Element(
            "testsuite",
            {
                "name": "dog2_locomotion_acceptance",
                "tests": "1",
                "failures": "0" if passed or infrastructure else "1",
                "errors": "1" if infrastructure else "0",
                "time": f"{report['timing']['wall_duration_sec']:.6f}",
            },
        )
        case = ET.SubElement(
            suite,
            "testcase",
            {
                "classname": "dog2_bringup",
                "name": str(report["trial_id"]),
                "time": f"{report['timing']['wall_duration_sec']:.6f}",
            },
        )
        if not passed:
            tag = "error" if infrastructure else "failure"
            failure = ET.SubElement(
                case,
                tag,
                {
                    "type": str(
                        (report.get("failure") or {}).get("code", report["status"])
                    ),
                    "message": str(report["message"]),
                },
            )
            failure.text = str(report.get("failure"))
        path = Path(str(self._params["junit_xml"]))
        path.parent.mkdir(parents=True, exist_ok=True)
        tree = ET.ElementTree(suite)
        with tempfile.NamedTemporaryFile(
            mode="wb",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as handle:
            tree.write(handle, encoding="utf-8", xml_declaration=True)
            temp_name = handle.name
        os.replace(temp_name, path)

    def _finish(self, success: bool, status: str, message: str) -> None:
        if self._done:
            return
        self._done = True
        self._publish_command(0.0, 0.0)
        self._close_stage()
        self._foot_metrics.finish()
        report = self._build_report(status, message)
        exit_code = 0 if success else (2 if status == "FAIL_INFRASTRUCTURE" else 1)
        try:
            atomic_write_json(str(self._params["result_json"]), report)
            self._write_samples()
            self._write_junit(report)
        except Exception as exc:
            exit_code = 2
            self.get_logger().error(f"Failed to write LAV1 report: {exc}")
        log = self.get_logger().info if success else self.get_logger().error
        log(f"{status}: {message}; report={self._params['result_json']}")
        timer = threading.Timer(0.25, lambda: os._exit(exit_code))
        timer.daemon = True
        timer.start()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocomotionAcceptanceNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        if not node._done:
            node._fail(
                "INTERRUPTED",
                "acceptance process interrupted",
                infrastructure=True,
                provenance="TEST_INFRASTRUCTURE",
            )
    finally:
        if not node._done:
            try:
                node.destroy_node()
            except Exception:
                pass
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
