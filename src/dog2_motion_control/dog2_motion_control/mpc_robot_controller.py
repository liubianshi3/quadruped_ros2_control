"""
dog2 MPC + WBC motion controller (effort-mode skeleton).

This node is a placeholder for the future MPC algorithm:
- It demonstrates a complete ROS 2 Node initialization.
- It wires up the expected inputs/outputs for effort-mode control.

TODO: Implement MPC optimization and convert MPC torques -> forward_command_controller commands.
"""

from __future__ import annotations

import time
import os
from typing import Optional

import numpy as np
import osqp
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory

from scipy import sparse

import pinocchio as pin

from .config_loader import ConfigLoader
from .joint_names import ALL_JOINT_NAMES


class ConvexMPC:
    """
    Sparse-form LTV SRBD MPC solved by OSQP.

    State:
      X = [Theta(3), p(3), omega(3), v(3), -g(1)]  => 13D

    Control:
      U_k = [F_lf(3), F_lh(3), F_rh(3), F_rf(3)] => 12D, each foot force is [Fx, Fy, Fz].
    """

    def __init__(
        self,
        dt: float = 0.02,
        N: int = 5,
        mass_kg: float = 11.8,
        inertia_diag: tuple[float, float, float] = (0.1, 0.2, 0.2),
        mu: float = 0.5,
        fz_min: float = 0.0,
        fz_max: float = 150.0,
    ) -> None:
        self.dt = float(dt)
        self.N = int(N)
        self.nx = 13
        self.nu = 12

        self.mass_kg = float(mass_kg)
        self.I_inv = np.diag(
            [1.0 / inertia_diag[0], 1.0 / inertia_diag[1], 1.0 / inertia_diag[2]]
        ).astype(float)

        self.mu = float(mu)
        self.fz_min = float(fz_min)
        self.fz_max = float(fz_max)

        # Q = diag([10, 10, 10, 50, 50, 50, 1, 1, 1, 1, 1, 1, 0])
        self.Q_diag = np.array(
            [10.0, 10.0, 10.0, 80.0, 80.0, 600.0, 2.0, 2.0, 2.0, 4.0, 4.0, 40.0, 0.0],
            dtype=float,
        )
        self.Q_sp = sparse.diags(self.Q_diag, offsets=0, format="csc")

        # Keep input cost much smaller than the short-horizon falling cost,
        # otherwise OSQP prefers "save effort and let the body drop".
        self.R_diag = (1e-6) * np.ones(self.nu, dtype=float)
        self.R_sp = sparse.diags(self.R_diag, offsets=0, format="csc")

        # Z = [x_1..x_N, u_0..u_{N-1}]
        self.n_state_vars = self.N * self.nx
        self.n_input_vars = self.N * self.nu
        self.nZ = self.n_state_vars + self.n_input_vars

        # Hessian P (OSQP standard form: min 0.5*z^T P z + q^T z)
        P_state = sparse.kron(sparse.eye(self.N, format="csc"), self.Q_sp, format="csc")
        P_input = sparse.kron(sparse.eye(self.N, format="csc"), self.R_sp, format="csc")
        self.P = sparse.bmat([[P_state, None], [None, P_input]], format="csc")
        self.P = self.P + 1e-9 * sparse.eye(self.nZ, format="csc")  # small numerical regularization

        # Static input-only inequality constraints in OSQP form:
        #   l_ineq <= A_ineq * Z <= u_ineq
        A_step, l_step, u_step = self._build_input_constraints_for_one_step()
        A_step_sp = sparse.csc_matrix(A_step)  # (24 x 12)

        A_uineq = sparse.kron(sparse.eye(self.N, format="csc"), A_step_sp, format="csc")  # (N*24) x (N*12)
        zeros_x = sparse.csc_matrix((self.N * A_step.shape[0], self.n_state_vars))  # (N*24) x (N*13)
        self.A_ineq = sparse.hstack([zeros_x, A_uineq], format="csc")
        self.l_ineq = np.tile(l_step, self.N).astype(float)
        self.u_ineq_base = np.tile(u_step, self.N).astype(float)
        self.u_ineq = self.u_ineq_base  # backward compat inside this file

        # Safe fallback buffer
        self._last_u_opt = np.zeros((self.N, self.nu), dtype=float)

        # QP failure guardrails (avoid repeated "all-zero GRF" sequences)
        self._qp_fail_count = 0
        self._qp_fail_limit = 3
        self.last_used_fallback: bool = False
        self.last_status_val: int = -1
        self._last_contact01: Optional[np.ndarray] = None

        # Reuse a single OSQP instance
        self._osqp_solver = osqp.OSQP()

        # Indices (in the OSQP inequality bounds vector `u`) corresponding to:
        #   Fz <= fz_max
        # with constraint ordering inside `_build_input_constraints_for_one_step`:
        #   for each leg: [Fx - mu*Fz, -Fx - mu*Fz, Fy - mu*Fz, -Fy - mu*Fz, Fz<=fz_max, -Fz<=-fz_min]
        # So the row within each (leg) block is `leg*6 + 4`.
        self._fz_upper_indices = np.zeros((self.N, 4), dtype=int)
        constraints_per_leg = 6
        cons_per_step = 4 * constraints_per_leg  # 24
        for k in range(self.N):
            for leg in range(4):
                self._fz_upper_indices[k, leg] = k * cons_per_step + leg * constraints_per_leg + 4

    def _build_nominal_support_u(self, contact_now: np.ndarray) -> np.ndarray:
        """
        Conservative non-zero stance GRF sequence as a last-resort fallback.
        """
        contact_now = np.asarray(contact_now, dtype=float).reshape(4)
        u = np.zeros((self.N, self.nu), dtype=float)
        n_stance = max(1, int(np.sum(contact_now > 0.5)))
        fz_nom = self.mass_kg * 9.81 / float(n_stance)
        fz_nom = float(np.clip(fz_nom, self.fz_min, self.fz_max))
        for k in range(self.N):
            for leg in range(4):
                if bool(contact_now[leg] > 0.5):
                    u[k, 3 * leg + 2] = fz_nom
        return u

    def _build_ac(self, psi: float) -> np.ndarray:
        """
        Continuous-time SRBD linearized A_c, using:
          p_dot = v
          Theta_dot = Rz(psi) * omega
          v_dot has gravity via the state element "-g" (scalar)
        """
        Ac = np.zeros((self.nx, self.nx), dtype=float)

        # State indexing
        # Theta: 0..2
        # p: 3..5
        # omega: 6..8
        # v: 9..11
        # -g scalar: 12

        # p_dot = v
        Ac[3:6, 9:12] = np.eye(3, dtype=float)

        # Theta_dot = Rz(psi) * omega
        c = float(np.cos(psi))
        s = float(np.sin(psi))
        Rz = np.array(
            [
                [c, -s, 0.0],
                [s, c, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        Ac[0:3, 6:9] = Rz

        # v_dot = g + (1/m) sum F_i, where gravity is encoded by state x[-g] (scalar).
        # We assume gravity acts only on the world Z acceleration component:
        #   v_z_dot += (-g_state)
        # -> v_z_dot depends on state index 12 directly with coefficient +1.
        Ac[11, 12] = 1.0

        return Ac

    @staticmethod
    def _skew_symmetric(r: np.ndarray) -> np.ndarray:
        """Return skew(r) such that skew(r) @ f = r x f."""
        rx, ry, rz = [float(v) for v in r]
        return np.array(
            [
                [0.0, -rz, ry],
                [rz, 0.0, -rx],
                [-ry, rx, 0.0],
            ],
            dtype=float,
        )

    def _build_bc(self, r_foot: np.ndarray, psi: float) -> np.ndarray:
        """
        Continuous-time SRBD input matrix B_c.

        Controls are foot forces in world coordinates:
          U = [F_lf, F_lh, F_rh, F_rf] with each F_i = [Fx, Fy, Fz] (3D).

        Main blocks:
          omega_dot = I^{-1} sum_i (r_i x F_i)
          v_dot = (1/m) sum_i F_i
        """
        r_foot = np.asarray(r_foot, dtype=float)
        if r_foot.shape != (4, 3):
            raise ValueError(f"r_foot must be shape (4,3), got {r_foot.shape}")

        Bc = np.zeros((self.nx, self.nu), dtype=float)

        # World-frame inertia inverse at current yaw
        c = float(np.cos(psi))
        s = float(np.sin(psi))
        Rz = np.array(
            [
                [c, -s, 0.0],
                [s, c, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        Iw_inv = Rz @ self.I_inv @ Rz.T

        # omega rows: 6..8
        # v rows: 9..11
        for i in range(4):
            ri = r_foot[i, :]
            col_base = 3 * i

            # omega_dot contribution in world frame
            Bc[6:9, col_base : col_base + 3] = Iw_inv @ self._skew_symmetric(ri)

            # v_dot contribution: (1/m) * I3 * F_i
            Bc[9:12, col_base : col_base + 3] = (1.0 / self.mass_kg) * np.eye(3, dtype=float)

        return Bc

    def _build_input_constraints_for_one_step(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        n_legs = 4
        constraints_per_leg = 6
        n_cons_step = n_legs * constraints_per_leg

        A_step = np.zeros((n_cons_step, self.nu), dtype=float)
        l_step = -np.inf * np.ones(n_cons_step, dtype=float)
        u_step = np.zeros(n_cons_step, dtype=float)

        for leg_idx in range(4):
            col_base = 3 * leg_idx
            row_base = constraints_per_leg * leg_idx

            # Fx - mu*Fz <= 0
            A_step[row_base + 0, col_base + 0] = 1.0
            A_step[row_base + 0, col_base + 2] = -self.mu
            u_step[row_base + 0] = 0.0

            # -Fx - mu*Fz <= 0
            A_step[row_base + 1, col_base + 0] = -1.0
            A_step[row_base + 1, col_base + 2] = -self.mu
            u_step[row_base + 1] = 0.0

            # Fy - mu*Fz <= 0
            A_step[row_base + 2, col_base + 1] = 1.0
            A_step[row_base + 2, col_base + 2] = -self.mu
            u_step[row_base + 2] = 0.0

            # -Fy - mu*Fz <= 0
            A_step[row_base + 3, col_base + 1] = -1.0
            A_step[row_base + 3, col_base + 2] = -self.mu
            u_step[row_base + 3] = 0.0

            # Fz <= fz_max
            A_step[row_base + 4, col_base + 2] = 1.0
            u_step[row_base + 4] = self.fz_max

            # -Fz <= -fz_min  (=> Fz >= fz_min)
            A_step[row_base + 5, col_base + 2] = -1.0
            u_step[row_base + 5] = -self.fz_min

        return A_step, l_step, u_step

    def solve(
        self,
        x0: np.ndarray,
        r_foot_pred: np.ndarray,
        x_des_pred: np.ndarray,
        contact_pred: np.ndarray,
    ) -> np.ndarray:
        """
        Sparse-form LTV MPC solve with OSQP.

        Variable:
          Z = [x_1..x_N, u_0..u_{N-1}] in R^{N*(nx+nu)}

        Dynamics constraints:
          x_{k+1} - A_k x_k - B_k u_k = 0  (x_0 is known constant)

        Tracking:
          q_state = -Q * x_des,k  (for x_{k+1} block)
        """
        x0 = np.asarray(x0, dtype=float).reshape(self.nx)
        r_foot_pred = np.asarray(r_foot_pred, dtype=float)
        x_des_pred = np.asarray(x_des_pred, dtype=float)
        contact_pred = np.asarray(contact_pred, dtype=float)

        if r_foot_pred.shape != (self.N, 4, 3):
            raise ValueError(f"r_foot_pred must be shape (N,4,3)=({self.N},4,3), got {r_foot_pred.shape}")
        if x_des_pred.shape != (self.N, self.nx):
            raise ValueError(f"x_des_pred must be shape (N,13)=({self.N},13), got {x_des_pred.shape}")
        if contact_pred.shape != (self.N, 4):
            raise ValueError(
                f"contact_pred must be shape (N,4)=({self.N},4), got {contact_pred.shape}"
            )

        I_nx = np.eye(self.nx, dtype=float)
        A_list: list[np.ndarray] = []
        B_list: list[np.ndarray] = []

        # LTV discretization (2nd-order ZOH truncation)
        for k in range(self.N):
            psi_des = float(x_des_pred[k, 2])
            Ac = self._build_ac(psi_des)
            Bc = self._build_bc(r_foot_pred[k], psi_des)

            Ad = I_nx + Ac * self.dt + 0.5 * (Ac @ Ac) * (self.dt**2)
            Bd = (I_nx * self.dt + 0.5 * Ac * (self.dt**2)) @ Bc

            A_list.append(Ad)
            B_list.append(Bd)

        # Gradient q:
        #   q = [-Q x_des,1, ...,-Q x_des,N, 0,...,0]^T
        q = np.zeros(self.nZ, dtype=float)
        for k in range(self.N):
            q[k * self.nx : (k + 1) * self.nx] = -self.Q_diag * x_des_pred[k, :]

        # Equality constraints A_eq * Z = b_eq
        # A_eq: (N*nx) x (N*(nx+nu)) with the block structure:
        #   row k (x_{k+1}):
        #     +I on x_{k+1}
        #     -A_k on x_k (k>0, x_k variable)
        #     -B_k on u_k
        #     b_eq[0] = A_0 x0 else 0
        I_nx_sp = sparse.eye(self.nx, format="csc")
        row_blocks: list[list[sparse.spmatrix | None]] = []
        for k in range(self.N):
            row: list[sparse.spmatrix | None] = [None] * (2 * self.N)
            row[k] = I_nx_sp  # coefficient for x_{k+1}
            if k > 0:
                row[k - 1] = sparse.csc_matrix(-A_list[k])  # coefficient for x_k
            row[self.N + k] = sparse.csc_matrix(-B_list[k])  # coefficient for u_k
            row_blocks.append(row)

        A_eq = sparse.bmat(row_blocks, format="csc")

        b_eq = np.zeros(self.N * self.nx, dtype=float)
        b_eq[0 : self.nx] = A_list[0] @ x0

        # Combine equality + inequality constraints
        # Inequality constraints are:
        #   l_ineq <= A_ineq*Z <= u_ineq
        # Only the `Fz <= fz_max` upper bounds are time/leg dependent under contact schedule.
        A = sparse.vstack([A_eq, self.A_ineq], format="csc")
        l = np.concatenate([b_eq, self.l_ineq]).astype(float)

        u = np.concatenate([b_eq, self.u_ineq_base.copy()]).astype(float)

        # Dynamic contact constraints: synchronize both Fz upper/lower bounds.
        # Constraint ordering per leg:
        #   [Fx-mu*Fz, -Fx-mu*Fz, Fy-mu*Fz, -Fy-mu*Fz, Fz<=fz_max, -Fz<=-fz_min]
        contact01 = (contact_pred > 0.5).astype(float)
        contact_schedule_changed = False
        if self._last_contact01 is None:
            contact_schedule_changed = True
        else:
            # Any stance/swing flip across the horizon is a hard constraint change.
            contact_schedule_changed = bool(np.any(contact01 != self._last_contact01))
        self._last_contact01 = contact01.copy()

        offset = self.N * self.nx
        for k in range(self.N):
            for leg in range(4):
                stance = float(contact01[k, leg])
                idx_base = offset + k * 24 + leg * 6
                u[idx_base + 4] = stance * self.fz_max
                u[idx_base + 5] = -stance * self.fz_min

        # Solve QP
        self._osqp_solver.setup(
            P=self.P,
            q=q,
            A=A,
            l=l,
            u=u,
            verbose=False,
            warm_start=not contact_schedule_changed,
            polish=True,
            eps_abs=1e-3,
            eps_rel=1e-3,
            max_iter=4000 if contact_schedule_changed else 20000,
        )
        res = self._osqp_solver.solve()

        status_val = int(getattr(res.info, "status_val", -1))
        self.last_status_val = status_val
        if getattr(res, "x", None) is not None and status_val in (1, 2):
            self._qp_fail_count = 0
            self.last_used_fallback = False
            z_opt = np.asarray(res.x, dtype=float)
            u_stack = z_opt[self.n_state_vars : self.nZ]
            u_seq = u_stack.reshape(self.N, self.nu)
            self._last_u_opt = u_seq
            return u_seq

        self._qp_fail_count += 1
        if int(self._qp_fail_count) >= int(self._qp_fail_limit):
            u_nom = self._build_nominal_support_u(contact_pred[0, :])
            self._last_u_opt = u_nom
            self.last_used_fallback = True
            return u_nom

        # Safe fallback: shift previous optimal inputs
        u_fallback = self._last_u_opt.copy()
        if self.N >= 2:
            u_fallback[0 : self.N - 1, :] = self._last_u_opt[1 : self.N, :]
        u_fallback[self.N - 1, :] = self._last_u_opt[self.N - 1, :]

        # Enforce contact schedule on the fallback (swing legs => zero forces).
        # This keeps the controller behavior consistent even when OSQP fails.
        contact01 = contact_pred > 0.5
        for k in range(self.N):
            for leg in range(4):
                idx = 3 * leg
                if not bool(contact01[k, leg]):
                    u_fallback[k, idx : idx + 3] = 0.0
                    continue

                fz = float(u_fallback[k, idx + 2])
                fz = float(np.clip(fz, self.fz_min, self.fz_max))
                u_fallback[k, idx + 2] = fz
                max_xy = self.mu * fz
                u_fallback[k, idx + 0] = float(np.clip(float(u_fallback[k, idx + 0]), -max_xy, max_xy))
                u_fallback[k, idx + 1] = float(np.clip(float(u_fallback[k, idx + 1]), -max_xy, max_xy))

        self.last_used_fallback = True
        return u_fallback


class MPCRobotController(Node):
    """Effort-mode MPC controller node skeleton (no MPC algorithm yet)."""

    def __init__(self) -> None:
        super().__init__("mpc_robot_controller")
        self.get_logger().info("Initializing MPC robot controller...")

        self.declare_parameter("config_file", "")
        self.declare_parameter("control_period_sec", 0.02)  # ~50 Hz
        self.declare_parameter("effort_controller_topic", "/effort_controller/commands")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_twist_in_local_frame", True)
        self.declare_parameter("debug_mode", False)
        self.declare_parameter("startup_full_stance_duration_sec", 3.0)
        self.declare_parameter("nominal_base_height", 0.24)
        # 关节 PD 站立阶段：在仿真中原始增益偏硬，容易在脚先落地后产生高频“跺地”抽搐。
        # 这里整体软化关节刚度、略提高阻尼，优先让机器人“稳稳趴住”，后续再逐步加硬。
        self.declare_parameter("rail_lock_kp", 800.0)
        self.declare_parameter("rail_lock_kd", 140.0)
        self.declare_parameter("idle_cmd_threshold", 0.02)
        self.declare_parameter("standup_joint_error_tolerance_rad", 0.18)
        self.declare_parameter("standup_joint_velocity_tolerance_rad_s", 0.75)
        # Standup 阶段的两类“防抽搐”处理：
        # 1) 期望姿态 ramp：避免一开始就用高 kp 去硬顶碰撞约束
        # 2) 穿地检测：如果足端仍在地面以下，则临时软化 kp、加大 kd
        self.declare_parameter("standup_ramp_time_sec", 1.0)
        self.declare_parameter("standup_penetration_foot_z_threshold_m", 0.0)
        self.declare_parameter("standup_penetration_kp_scale", 0.35)
        self.declare_parameter("standup_penetration_kd_scale", 1.3)
        self.declare_parameter("standup_use_gravity_compensation", False)
        self.declare_parameter("standup_tau_abs_limit", 35.0)
        self.declare_parameter("idle_hold_coxa_kp", 40.0)
        self.declare_parameter("idle_hold_coxa_kd", 8.0)
        self.declare_parameter("idle_hold_femur_kp", 70.0)
        self.declare_parameter("idle_hold_femur_kd", 14.0)
        self.declare_parameter("idle_hold_tibia_kp", 70.0)
        self.declare_parameter("idle_hold_tibia_kd", 14.0)
        self.declare_parameter("use_gz_foot_contact", False)
        self.declare_parameter("gz_contact_topic_lf", "/dog2/foot_contact/lf")
        self.declare_parameter("gz_contact_topic_lh", "/dog2/foot_contact/lh")
        self.declare_parameter("gz_contact_topic_rh", "/dog2/foot_contact/rh")
        self.declare_parameter("gz_contact_topic_rf", "/dog2/foot_contact/rf")

        self._config_file: str = str(self.get_parameter("config_file").value)
        self._control_period_sec: float = float(self.get_parameter("control_period_sec").value)
        self._effort_controller_topic: str = str(self.get_parameter("effort_controller_topic").value)
        self._odom_topic: str = str(self.get_parameter("odom_topic").value)
        self._odom_twist_in_local_frame: bool = bool(self.get_parameter("odom_twist_in_local_frame").value)
        self._debug_mode: bool = bool(self.get_parameter("debug_mode").value)
        self._startup_full_stance_duration_sec: float = float(
            self.get_parameter("startup_full_stance_duration_sec").value
        )
        self._nominal_base_height: float = float(self.get_parameter("nominal_base_height").value)
        self._rail_lock_kp: float = float(self.get_parameter("rail_lock_kp").value)
        self._rail_lock_kd: float = float(self.get_parameter("rail_lock_kd").value)
        self._idle_cmd_threshold: float = float(self.get_parameter("idle_cmd_threshold").value)
        self._standup_joint_error_tolerance_rad: float = float(
            self.get_parameter("standup_joint_error_tolerance_rad").value
        )
        self._standup_joint_velocity_tolerance_rad_s: float = float(
            self.get_parameter("standup_joint_velocity_tolerance_rad_s").value
        )
        self._standup_ramp_time_sec: float = float(self.get_parameter("standup_ramp_time_sec").value)
        self._standup_penetration_foot_z_threshold_m: float = float(
            self.get_parameter("standup_penetration_foot_z_threshold_m").value
        )
        self._standup_penetration_kp_scale: float = float(
            self.get_parameter("standup_penetration_kp_scale").value
        )
        self._standup_penetration_kd_scale: float = float(
            self.get_parameter("standup_penetration_kd_scale").value
        )
        self._standup_use_gravity_compensation: bool = bool(
            self.get_parameter("standup_use_gravity_compensation").value
        )
        self._standup_tau_abs_limit: float = float(
            self.get_parameter("standup_tau_abs_limit").value
        )
        self._idle_hold_coxa_kp: float = float(self.get_parameter("idle_hold_coxa_kp").value)
        self._idle_hold_coxa_kd: float = float(self.get_parameter("idle_hold_coxa_kd").value)
        self._idle_hold_femur_kp: float = float(self.get_parameter("idle_hold_femur_kp").value)
        self._idle_hold_femur_kd: float = float(self.get_parameter("idle_hold_femur_kd").value)
        self._idle_hold_tibia_kp: float = float(self.get_parameter("idle_hold_tibia_kp").value)
        self._idle_hold_tibia_kd: float = float(self.get_parameter("idle_hold_tibia_kd").value)
        self._use_gz_foot_contact: bool = bool(self.get_parameter("use_gz_foot_contact").value)
        self._gz_foot_in_contact = np.ones(4, dtype=int)
        self._startup_time = self.get_clock().now()
        self._standup_complete = False

        self._target_twist = (0.0, 0.0, 0.0)  # (vx, vy, omega)
        self._last_cmd_vel_time: Optional[Time] = None

        self._last_joint_state: Optional[JointState] = None
        self._last_joint_state_time: Optional[Time] = None
        # Standup ramp 起点（pin q 空间）；仅在第一次进入 joint_pd_standup 时记录
        self._standup_q_start: Optional[np.ndarray] = None

        # Real base state from odometry:
        # pose = [x, y, z, qx, qy, qz, qw], twist = [vx, vy, vz, wx, wy, wz]
        self._real_base_pose = np.array([0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0], dtype=float)
        # Raw odom twist (commonly in base/local frame).
        self._real_base_twist = np.zeros(6, dtype=float)
        # World-frame twist used by SRBD state and free-flyer velocity.
        self._real_base_twist_world = np.zeros(6, dtype=float)
        self._last_odom_time: Optional[Time] = None

        # Inputs (placeholders for future state estimation / MPC feedback).
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)

        if self._use_gz_foot_contact:
            gz_topics = (
                str(self.get_parameter("gz_contact_topic_lf").value),
                str(self.get_parameter("gz_contact_topic_lh").value),
                str(self.get_parameter("gz_contact_topic_rh").value),
                str(self.get_parameter("gz_contact_topic_rf").value),
            )
            for leg_idx, topic in enumerate(gz_topics):
                self.create_subscription(
                    Contacts,
                    topic,
                    self._make_gz_foot_contact_cb(leg_idx),
                    10,
                )
            self.get_logger().info(
                "Gazebo foot contact fusion enabled (topics lf,lh,rh,rf order matches MPC leg order)."
            )

        # Output for forward_command_controller (effort commands).
        self._effort_pub = self.create_publisher(Float64MultiArray, self._effort_controller_topic, 10)

        # === 固化输出通道的关节顺序 (必须与 effort_controllers.yaml 完全一致!) ===
        self._ordered_joint_names = list(ALL_JOINT_NAMES)

        # 建立反向查找字典，确保填装数组时按名索引，永不出错
        self._joint_index_map = {name: idx for idx, name in enumerate(self._ordered_joint_names)}
        self._rail_joint_names = [name for name in self._ordered_joint_names if name.endswith("_rail_joint")]
        self._rail_hold_targets = {name: 0.0 for name in self._rail_joint_names}
        self._standing_joint_targets = {name: 0.0 for name in self._ordered_joint_names}

        # Defaults for crawl schedule (will be overwritten by gait_params.yaml when available).
        self._gait_cycle_time_sec = 1.2
        self._gait_duty_factor = 0.75
        self._gait_stride_height = 0.03
        self._gait_type = "crawl"

        try:
            cfg = ConfigLoader(self._config_file if self._config_file else None)
            cfg.load()
            gait_cfg = cfg.get_gait_config()
            self._nominal_base_height = float(gait_cfg.body_height)
            self._gait_cycle_time_sec = float(gait_cfg.cycle_time)
            self._gait_duty_factor = float(gait_cfg.duty_factor)
            self._gait_stride_height = float(gait_cfg.stride_height)
            self._gait_type = str(gait_cfg.gait_type)
            # Match swing parabola peak with the gait generator's stride_height.
            self._swing_clearance = float(max(0.005, self._gait_stride_height))

            standing_pose = cfg.get_config_data().get("standing_pose", {})
            for leg_prefix in ("lf", "lh", "rh", "rf"):
                rail_joint_name = f"{leg_prefix}_rail_joint"
                coxa_joint_name = f"{leg_prefix}_coxa_joint"
                femur_joint_name = f"{leg_prefix}_femur_joint"
                tibia_joint_name = f"{leg_prefix}_tibia_joint"
                leg_pose = standing_pose.get(leg_prefix, {})
                if "rail_m" in leg_pose:
                    rail_target = float(leg_pose["rail_m"])
                    self._rail_hold_targets[rail_joint_name] = rail_target
                    self._standing_joint_targets[rail_joint_name] = rail_target
                if "hip_roll_rad" in leg_pose:
                    self._standing_joint_targets[coxa_joint_name] = float(leg_pose["hip_roll_rad"])
                if "hip_pitch_rad" in leg_pose:
                    self._standing_joint_targets[femur_joint_name] = float(leg_pose["hip_pitch_rad"])
                if "knee_pitch_rad" in leg_pose:
                    self._standing_joint_targets[tibia_joint_name] = float(leg_pose["knee_pitch_rad"])
        except Exception as exc:
            self.get_logger().warn(f"Failed to load MPC runtime config details: {exc}")

        # -----------------------------
        # Pinocchio: floating-base model
        # -----------------------------
        # Pinocchio model source:
        # - If `urdf_xml` is provided, we directly build from XML string.
        # - Otherwise, we prefer expanding the canonical xacro file (guarantees joint names
        #   like lf_rail_joint/... are present) and build from the resulting URDF XML.
        # - `urdf_path` is kept as an explicit override for pre-expanded URDF snapshots.
        default_urdf_path = ""
        default_urdf_xacro_path = os.path.join(
            get_package_share_directory("dog2_description"),
            "urdf",
            "dog2.urdf.xacro",
        )
        default_controllers_yaml_path = os.path.join(
            get_package_share_directory("dog2_motion_control"),
            "config",
            "effort_controllers.yaml",
        )

        self.declare_parameter("urdf_path", default_urdf_path)
        self.declare_parameter("urdf_xml", "")
        self.declare_parameter("urdf_xacro_path", default_urdf_xacro_path)
        self.declare_parameter("controllers_yaml_path", default_controllers_yaml_path)
        self.declare_parameter("mass_scale", 1.0)
        self.declare_parameter("control_mode", "effort")

        self._pin_model = None
        self._pin_data = None
        self._pin_q: Optional[np.ndarray] = None
        self._pin_v: Optional[np.ndarray] = None
        self._pin_q_ready = False

        urdf_path = str(self.get_parameter("urdf_path").value)
        urdf_xml = str(self.get_parameter("urdf_xml").value)
        urdf_xacro_path = str(self.get_parameter("urdf_xacro_path").value)
        controllers_yaml_path = str(self.get_parameter("controllers_yaml_path").value)
        mass_scale = str(self.get_parameter("mass_scale").value)
        control_mode = str(self.get_parameter("control_mode").value)

        if urdf_xml.strip():
            self._pin_model = pin.buildModelFromXML(urdf_xml, pin.JointModelFreeFlyer())
        elif urdf_xacro_path.strip():
            # Expand xacro inside the node to keep pinocchio joint naming consistent
            # with gz_ros2_control's robot_description.
            import xacro  # local import to keep startup lightweight

            doc = xacro.process_file(
                urdf_xacro_path,
                mappings={
                    "controllers_yaml": controllers_yaml_path,
                    "mass_scale": mass_scale,
                    "control_mode": control_mode,
                },
            )
            self._pin_model = pin.buildModelFromXML(doc.toxml(), pin.JointModelFreeFlyer())
        elif urdf_path.strip():
            self._pin_model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
        else:
            raise RuntimeError(
                "No URDF source configured. Set one of urdf_xml, urdf_xacro_path, or urdf_path."
            )

        self._pin_data = self._pin_model.createData()

        # 获取 Pinocchio 内部的关节名称列表：model.names
        self._pin_model_names = [str(n) for n in self._pin_model.names]
        self.get_logger().info(
            "Pinocchio model initialized: "
            f"nq={self._pin_model.nq}, nv={self._pin_model.nv}, "
            f"names_count={len(self._pin_model_names)}"
        )

        if int(self._pin_model.nq) != 23:
            self.get_logger().warn(
                f"Expected pinocchio nq=23 (7 + 16 motors), but got nq={self._pin_model.nq}. "
                "Gravity compensation may map incorrectly."
            )
        if int(self._pin_model.nv) != 22:
            self.get_logger().warn(
                f"Expected pinocchio nv=22 (6 + 16 motors), but got nv={self._pin_model.nv}. "
                "Jacobian/torque mapping may be inconsistent."
            )

        # 状态映射：ROS joint_states.name -> Pinocchio joint id
        self._joint_state_name_to_pin_joint_id: dict[str, int] = {}
        self._pin_q_idx_by_motor: dict[str, int] = {}
        self._pin_v_idx_by_motor: dict[str, int] = {}

        for joint_name in self._ordered_joint_names:
            jid = int(self._pin_model.getJointId(joint_name))
            self._joint_state_name_to_pin_joint_id[joint_name] = jid

            joint = self._pin_model.joints[jid]
            self._pin_q_idx_by_motor[joint_name] = int(joint.idx_q)
            self._pin_v_idx_by_motor[joint_name] = int(joint.idx_v)

            # Motor coordinates should be after the FreeFlyer 7D pose.
            if int(joint.idx_q) < 7:
                self.get_logger().warn(
                    f"Motor joint '{joint_name}' has idx_q={joint.idx_q} (<7). "
                    "Please check URDF joint ordering / model building."
                )

        # Foot/contact frame ids used by SRBD -> whole-body torque mapping.
        self._foot_frame_names = {
            "lf": "lf_foot_link",
            "lh": "lh_foot_link",
            "rh": "rh_foot_link",
            "rf": "rf_foot_link",
        }
        self._foot_frame_ids = {leg: int(self._pin_model.getFrameId(name)) for leg, name in self._foot_frame_names.items()}
        for leg, frame_name in self._foot_frame_names.items():
            frame_id = self._foot_frame_ids[leg]
            if frame_id >= int(self._pin_model.nframes):
                raise RuntimeError(f"Foot frame '{frame_name}' not found in pinocchio model.")

        self._timer = self.create_timer(self._control_period_sec, self._on_control_timer)
        self._last_debug_log_walltime = time.time()

        # Instantiate convex MPC QP builder + OSQP solver.
        # (Keeping this separate from the URDF/odom logic keeps the node code tidy.)
        self.mpc_solver = ConvexMPC(dt=0.02, N=5)

        # ---- Gait / contact schedule (trot) + swing foot PD ----
        # Leg order must match `self._ordered_joint_names` mapping and the MPC force ordering:
        #   idx: 0=lf, 1=lh, 2=rh, 3=rf
        self._leg_order = ("lf", "lh", "rh", "rf")
        # Use the same crawl contact schedule idea as `GaitGenerator`:
        # - duty_factor >= 0.75 => 1 leg swings at a time => at least 3 legs stance
        # - this keeps a support triangle during idle/slow standing
        self._crawl_phase_offsets = {"lf": 0.50, "lh": 0.75, "rh": 0.00, "rf": 0.25}
        self._gait_step_count = 0
        # Start at global phase=0 with offsets above:
        # lf stance, lh stance, rh swing, rf stance => stance mask [lf, lh, rh, rf]=[1,1,0,1]
        self._last_contact_now = np.array([1, 1, 0, 1], dtype=int)
        self._swing_start_r = np.zeros((4, 3), dtype=float)
        self._swing_end_r = np.zeros((4, 3), dtype=float)
        # Discrete step index when each leg entered swing (used for per-leg swing_alpha).
        # -1 means "unknown / not initialized yet".
        self._swing_liftoff_step = np.full((4,), -1, dtype=int)

        # Swing trajectory (in base-relative coordinates): z follows a parabola.
        # Default will be overwritten by YAML gait params (stride_height) when config loading succeeds.
        self._swing_clearance = 0.03
        self._step_length_scale = 0.6

        # Raibert foot placement terms:
        #   p_foot = p_hip
        #          + 0.5 * v_cmd * T_stance
        #          + k_v * (v_actual - v_cmd)
        #          + sqrt(h/g) * (v_actual x omega_actual)
        # We use XY components for foothold update while keeping current Z touchdown policy.
        self._raibert_kv_xy = np.array([0.08, 0.08], dtype=float)
        self._gravity_mag = 9.81
        self._min_base_height_for_raibert = 0.12

        # Odometry twist spike hardening (LPF before Raibert).
        # This is intentionally separate from the Raibert delta LPF:
        # - twist LPF: removes contact-induced velocity spikes from odom
        # - delta LPF: removes residual discrete foothold jumps
        self._v_world_filt = np.zeros(3, dtype=float)
        self._w_world_filt = np.zeros(3, dtype=float)
        self._twist_lpf_tau = 0.06

        # Cartesian PD gains applied to swing feet (world coordinates).
        # Tune if needed for your Gazebo contact parameters.
        self._Kp_swing = np.array([250.0, 250.0, 600.0], dtype=float)
        self._Kd_swing = np.array([25.0, 25.0, 60.0], dtype=float)
        self._F_pd_min = np.array([-120.0, -120.0, -200.0], dtype=float)
        self._F_pd_max = np.array([120.0, 120.0, 200.0], dtype=float)
        self._tau_abs_limit = 80.0

        # ---- Safety guardrails for stance<->swing transitions ----
        # Contact blending (0=swing, 1=stance) prevents MPC/PD double actuation
        # when contact schedule and real contact are misaligned by 1-2 frames.
        self._contact_blend_steps = 2
        self._contact_blend = self._last_contact_now.astype(float).copy()

        # ---- Raibert signal conditioning ----
        self._raibert_v_max_xy = np.array([2.5, 2.5], dtype=float)  # m/s clamp on odom spikes
        self._raibert_delta_max_xy = np.array([0.08, 0.08], dtype=float)  # m clamp per step (hard safety)
        self._raibert_lpf_tau_sec = 0.10  # LPF time constant for delta_xy
        self._raibert_delta_xy_filt = np.zeros(2, dtype=float)
        self._freeze_raibert_when_unstable = True
        self._freeze_min_base_height = 0.10
        self._freeze_max_abs_rp_rad = float(np.deg2rad(55.0))

        # ---- Fallback damping in QP failure ----
        # When MPC fails, inject a conservative horizontal damping GRF to avoid "control holes".
        self._fallback_kd_xy = np.array([40.0, 40.0], dtype=float)  # N per (m/s) total
        self._fallback_kd_omega_z = 6.0  # N per (m/s) equivalent via simple distribution

        self.get_logger().info(
            "MPC controller ready. Waiting for /cmd_vel and /joint_states. "
            f"config_file='{self._config_file}', nominal_base_height={self._nominal_base_height:.3f}"
        )

        # Track whether we were in mpc_gait in the previous timer tick.
        # When switching into mpc_gait, we reset the gait phase counters.
        self._was_in_mpc_gait = False

    def _make_gz_foot_contact_cb(self, leg_idx: int):
        def _cb(msg: Contacts) -> None:
            self._gz_foot_in_contact[leg_idx] = 1 if len(msg.contacts) > 0 else 0

        return _cb

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._target_twist = (float(msg.linear.x), float(msg.linear.y), float(msg.angular.z))
        self._last_cmd_vel_time = self.get_clock().now()

    def _on_joint_state(self, msg: JointState) -> None:
        self._last_joint_state = msg
        self._last_joint_state_time = self.get_clock().now()
        self._update_pinocchio_q()

    def _on_odom(self, msg: Odometry) -> None:
        self._real_base_pose = np.array(
            [
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                float(msg.pose.pose.position.z),
                float(msg.pose.pose.orientation.x),
                float(msg.pose.pose.orientation.y),
                float(msg.pose.pose.orientation.z),
                float(msg.pose.pose.orientation.w),
            ],
            dtype=float,
        )

        self._real_base_twist = np.array(
            [
                float(msg.twist.twist.linear.x),
                float(msg.twist.twist.linear.y),
                float(msg.twist.twist.linear.z),
                float(msg.twist.twist.angular.x),
                float(msg.twist.twist.angular.y),
                float(msg.twist.twist.angular.z),
            ],
            dtype=float,
        )

        if self._odom_twist_in_local_frame:
            rot_world_from_base = self._quat_xyzw_to_rotmat(self._real_base_pose[3:7])
            v_local = self._real_base_twist[:3]
            omega_local = self._real_base_twist[3:]
            v_world = rot_world_from_base @ v_local
            omega_world = rot_world_from_base @ omega_local
            self._real_base_twist_world = np.concatenate([v_world, omega_world])
        else:
            self._real_base_twist_world = self._real_base_twist.copy()
        self._last_odom_time = self.get_clock().now()
        self._update_pinocchio_q()

    def _update_pinocchio_q(self) -> None:
        """Update pinocchio generalized coordinates (q, v) from odom + /joint_states."""
        if self._pin_model is None or self._last_joint_state is None:
            return

        # Strict dimensions for FreeFlyer + 16 motors.
        nq = int(self._pin_model.nq)
        nv = int(self._pin_model.nv)
        if nq != 23 or nv != 22:
            # Keep behavior predictable even if URDF changes.
            self._pin_q = np.zeros(nq, dtype=float)
            self._pin_v = np.zeros(nv, dtype=float)
            self._pin_q_ready = False
            return

        q = np.zeros(nq, dtype=float)
        v = np.zeros(nv, dtype=float)

        # q head: [x, y, z, qx, qy, qz, qw] from real odom.
        q[:7] = self._real_base_pose
        quat = q[3:7]
        quat_norm = float(np.linalg.norm(quat))
        if quat_norm < 1e-9:
            q[3:7] = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        else:
            q[3:7] = quat / quat_norm

        # v head for FreeFlyer must be world-aligned: [vx, vy, vz, wx, wy, wz].
        v[:3] = self._real_base_twist_world[:3]
        v[3:6] = self._real_base_twist_world[3:]

        name_to_pos = {str(n): float(p) for n, p in zip(self._last_joint_state.name, self._last_joint_state.position)}
        name_to_vel = {str(n): float(p) for n, p in zip(self._last_joint_state.name, self._last_joint_state.velocity)}
        missing = []

        for motor_name in self._ordered_joint_names:
            if motor_name not in name_to_pos:
                missing.append(motor_name)
                continue
            q_idx = self._pin_q_idx_by_motor[motor_name]
            v_idx = self._pin_v_idx_by_motor[motor_name]
            q[q_idx] = name_to_pos[motor_name]
            v[v_idx] = name_to_vel.get(motor_name, 0.0)

        if missing:
            if self._debug_mode:
                self.get_logger().warn(f"joint_states missing motors: {missing}")
            self._pin_q_ready = False
            return

        self._pin_q = q
        self._pin_v = v
        self._pin_q_ready = True

    def _quat_xyzw_to_rpy(self, q_xyzw: np.ndarray) -> np.ndarray:
        """Convert quaternion [x,y,z,w] to roll-pitch-yaw (XYZ intrinsic)."""
        x, y, z, w = [float(v) for v in q_xyzw]

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        sinp = float(np.clip(sinp, -1.0, 1.0))
        pitch = np.arcsin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return np.array([roll, pitch, yaw], dtype=float)

    def _quat_xyzw_to_rotmat(self, q_xyzw: np.ndarray) -> np.ndarray:
        """Convert quaternion [x,y,z,w] to rotation matrix (world-from-base)."""
        x, y, z, w = [float(v) for v in q_xyzw]
        n = np.sqrt(x * x + y * y + z * z + w * w)
        if n < 1e-9:
            return np.eye(3, dtype=float)
        x /= n
        y /= n
        z /= n
        w /= n

        xx = x * x
        yy = y * y
        zz = z * z
        xy = x * y
        xz = x * z
        yz = y * z
        wx = w * x
        wy = w * y
        wz = w * z

        return np.array(
            [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
                [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
                [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
            ],
            dtype=float,
        )

    def _cmd_vel_base_to_world_xy(self, yaw_world: float, vx_base: float, vy_base: float) -> np.ndarray:
        """Convert commanded planar velocity from base frame to world frame."""
        c = float(np.cos(yaw_world))
        s = float(np.sin(yaw_world))
        return np.array(
            [
                c * float(vx_base) - s * float(vy_base),
                s * float(vx_base) + c * float(vy_base),
            ],
            dtype=float,
        )

    def _compute_raibert_delta_xy(
        self,
        v_cmd_world_xy: np.ndarray,
        v_actual_world_xyz: np.ndarray,
        omega_actual_world_xyz: np.ndarray,
        stance_duration: float,
        base_height: float,
    ) -> np.ndarray:
        """
        Raibert foothold update in XY.

        Full heuristic:
          delta = 0.5 * v_cmd * T_stance
                + k_v * (v_actual - v_cmd)
                + sqrt(h/g) * (v_actual x omega_actual)
        """
        if v_cmd_world_xy.shape != (2,):
            raise ValueError(f"v_cmd_world_xy must be (2,), got {v_cmd_world_xy.shape}")
        if v_actual_world_xyz.shape != (3,):
            raise ValueError(f"v_actual_world_xyz must be (3,), got {v_actual_world_xyz.shape}")
        if omega_actual_world_xyz.shape != (3,):
            raise ValueError(f"omega_actual_world_xyz must be (3,), got {omega_actual_world_xyz.shape}")

        # 1) Symmetry/feedforward term
        sym_xy = 0.5 * float(max(stance_duration, 0.0)) * v_cmd_world_xy
        sym_xy *= float(self._step_length_scale)

        # 2) Velocity feedback term
        v_actual_xy = np.clip(v_actual_world_xyz[:2], -self._raibert_v_max_xy, self._raibert_v_max_xy)
        vel_fb_xy = self._raibert_kv_xy * (v_actual_xy - v_cmd_world_xy)

        # 3) Centrifugal/coriolis-like term
        h_eff = float(max(base_height, self._min_base_height_for_raibert))
        cross_term = np.cross(np.array([v_actual_xy[0], v_actual_xy[1], v_actual_world_xyz[2]], dtype=float), omega_actual_world_xyz)
        centrifugal_xy = np.sqrt(h_eff / self._gravity_mag) * cross_term[:2]

        delta_xy = sym_xy + vel_fb_xy + centrifugal_xy
        delta_xy = np.clip(delta_xy, -self._raibert_delta_max_xy, self._raibert_delta_max_xy)

        # 1st-order low-pass filter on the final delta (prevents touchdown jump on odom spikes).
        dt = float(self.mpc_solver.dt)
        tau = float(max(self._raibert_lpf_tau_sec, 1e-4))
        alpha = float(dt / (tau + dt))
        self._raibert_delta_xy_filt = (1.0 - alpha) * self._raibert_delta_xy_filt + alpha * delta_xy
        return self._raibert_delta_xy_filt.copy()

    def _update_contact_blend(self, contact_now: np.ndarray) -> None:
        """Advance per-leg blend factor towards {0,1} contact_now with a short ramp."""
        contact_now = np.asarray(contact_now, dtype=float).reshape(4)
        target = (contact_now > 0.5).astype(float)
        steps = float(max(int(self._contact_blend_steps), 1))
        rate = 1.0 / steps
        self._contact_blend += rate * (target - self._contact_blend)
        self._contact_blend = np.clip(self._contact_blend, 0.0, 1.0)

    def _solve_mpc_qp(self, x_state: np.ndarray, r_foot: np.ndarray, contact_pred: np.ndarray) -> np.ndarray:
        """
        Solve SRBD MPC QP (placeholder).

        SRBD state definition:
        X = [Theta^T, p^T, omega^T, v^T, -g]^T in R^13
          Theta: roll/pitch/yaw (3)
          p: base position (3)
          omega: base angular velocity (3)
          v: base linear velocity (3)
          -g: gravity scalar term (1)

        Control definition:
        U = [F_lf^T, F_lh^T, F_rh^T, F_rf^T]^T in R^12
          each F_* = [Fx, Fy, Fz].
        """
        # Build LTV sequences from current measurements/reference:
        #   r_foot_pred: (N,4,3)
        #   x_des_pred:  (N,13) reference over the horizon based on cmd_vel.
        N = self.mpc_solver.N

        x_state = np.asarray(x_state, dtype=float).reshape(13)
        yaw_now = float(x_state[2])

        vx_cmd_base, vy_cmd_base, omega_cmd = self._target_twist
        vx_cmd_base = float(vx_cmd_base)
        vy_cmd_base = float(vy_cmd_base)
        omega_cmd = float(omega_cmd)

        # State: [Theta(3), p(3), omega(3), v(3), -g(1)]
        x_des_pred = np.zeros((N, 13), dtype=float)
        dt = float(self.mpc_solver.dt)
        p_ref = x_state[3:6].copy()
        p_ref[2] = self._nominal_base_height
        for k in range(N):
            x_des = x_state.copy()

            # Orientation reference (track yaw command; keep roll/pitch near current)
            yaw_des_k = float(yaw_now + omega_cmd * float(k) * dt)
            x_des[0] = float(x_state[0])
            x_des[1] = float(x_state[1])
            x_des[2] = yaw_des_k

            # Angular velocity reference
            x_des[6:8] = 0.0
            x_des[8] = omega_cmd

            # Linear velocity reference in world frame, rotated by each predicted yaw.
            v_cmd_world_xy_k = self._cmd_vel_base_to_world_xy(yaw_des_k, vx_cmd_base, vy_cmd_base)
            x_des[9] = float(v_cmd_world_xy_k[0])
            x_des[10] = float(v_cmd_world_xy_k[1])
            x_des[11] = 0.0

            # Position reference integrates desired velocity for consistency with v_des.
            p_ref[0] += x_des[9] * dt
            p_ref[1] += x_des[10] * dt
            x_des[3:6] = p_ref

            # Maintain a nominal standing height instead of accepting the current collapsed height.
            z_err = float(self._nominal_base_height - float(x_state[5]))
            x_des[11] = float(np.clip(1.5 * z_err, -0.4, 0.4))

            # Gravity scalar term stays as-is (usually -9.81)
            x_des[12] = float(x_state[12])

            x_des_pred[k, :] = x_des
        r_foot_pred = np.tile(r_foot, (N, 1, 1))

        u_seq = self.mpc_solver.solve(x_state, r_foot_pred, x_des_pred, contact_pred)
        u0 = u_seq[0, :]
        if u0.shape != (12,):
            raise RuntimeError(f"Expected U_0 shape (12,), got {u0.shape}")
        return u0

    def _publish_efforts(self, target_efforts: dict[str, float]) -> None:
        """将包含关节名称的力矩字典，转换为固定顺序的 Float64MultiArray 并下发。"""
        msg = Float64MultiArray()
        msg.data = [0.0] * len(self._ordered_joint_names)

        for joint_name, effort_val in target_efforts.items():
            idx = self._joint_index_map.get(joint_name)
            if idx is None:
                self.get_logger().warn(f"未知的关节名称: {joint_name}，力矩将被丢弃!")
                continue
            msg.data[idx] = float(effort_val)

        self._effort_pub.publish(msg)

    def _joint_hold_gains(self, joint_name: str) -> tuple[float, float]:
        if joint_name.endswith("_rail_joint"):
            return float(self._rail_lock_kp), float(self._rail_lock_kd)
        if joint_name.endswith("_coxa_joint"):
            return float(self._idle_hold_coxa_kp), float(self._idle_hold_coxa_kd)
        if joint_name.endswith("_femur_joint"):
            return float(self._idle_hold_femur_kp), float(self._idle_hold_femur_kd)
        return float(self._idle_hold_tibia_kp), float(self._idle_hold_tibia_kd)

    def _joint_space_hold_tau(
        self,
        gravity: np.ndarray,
        ramp_alpha: float,
        min_foot_world_z: float,
    ) -> np.ndarray:
        """
        joint_pd_standup / idle_hold 的关节空间 PD（含重力补偿）。

        ramp_alpha: 期望关节从 ramp 起点逐步过渡到 standing_pose 的比例 [0,1]
        min_foot_world_z: 用于检测是否仍存在足端穿地/碰撞顶住的情况
        """
        tau_cmd = gravity.copy()

        penetrating = bool(min_foot_world_z < float(self._standup_penetration_foot_z_threshold_m))
        kp_scale = float(self._standup_penetration_kp_scale) if penetrating else 1.0
        kd_scale = float(self._standup_penetration_kd_scale) if penetrating else 1.0

        if self._standup_q_start is None:
            # 如果时序上外部没能初始化 ramp 起点，就退化为“直接目标姿态”
            self._standup_q_start = self._pin_q.copy() if self._pin_q is not None else None
        for joint_name in self._ordered_joint_names:
            q_idx = self._pin_q_idx_by_motor[joint_name]
            v_idx = self._pin_v_idx_by_motor[joint_name]
            q_cur = float(self._pin_q[q_idx])
            v_cur = float(self._pin_v[v_idx])
            q_des_target = float(self._standing_joint_targets.get(joint_name, 0.0))
            q_start = float(self._standup_q_start[q_idx]) if self._standup_q_start is not None else q_cur
            q_des = float(q_start + float(ramp_alpha) * (q_des_target - q_start))
            kp, kd = self._joint_hold_gains(joint_name)
            # 穿地时软化刚度、加大阻尼，避免“硬顶约束”导致抽搐。
            tau_cmd[v_idx] += (float(kp) * kp_scale) * (q_des - q_cur) - (float(kd) * kd_scale) * v_cur
        return tau_cmd

    def _standing_tracking_metrics(self) -> tuple[float, float]:
        max_pos_err = 0.0
        max_vel = 0.0
        for joint_name in self._ordered_joint_names:
            q_idx = self._pin_q_idx_by_motor[joint_name]
            v_idx = self._pin_v_idx_by_motor[joint_name]
            q_cur = float(self._pin_q[q_idx])
            v_cur = float(self._pin_v[v_idx])
            q_des = float(self._standing_joint_targets.get(joint_name, 0.0))
            max_pos_err = max(max_pos_err, abs(q_des - q_cur))
            max_vel = max(max_vel, abs(v_cur))
        return max_pos_err, max_vel

    def _on_control_timer(self) -> None:
        if self._last_joint_state is None:
            if self._debug_mode:
                self.get_logger().debug("Waiting for first /joint_states...")
            return
        # In some Gazebo configurations, /odom may not be bridged/published.
        # Proceed with the last available base pose/twist (initialized defaults) instead of
        # stalling and letting the robot collapse.
        if self._last_odom_time is None and self._debug_mode:
            self.get_logger().warn(f"No odometry received yet on '{self._odom_topic}'. Running in degraded mode.")

        if not self._pin_q_ready or self._pin_q is None or self._pin_v is None:
            if self._debug_mode:
                self.get_logger().debug("Pinocchio state (q,v) not ready; waiting for complete state inputs...")
            return

        # Build SRBD state: X = [Theta, p, omega, v, -g] in R^13.
        theta_rpy = self._quat_xyzw_to_rpy(self._real_base_pose[3:7])
        p_world = self._real_base_pose[:3].copy()
        omega_world = self._real_base_twist_world[3:].copy()
        v_world = self._real_base_twist_world[:3].copy()
        x_state = np.concatenate([theta_rpy, p_world, omega_world, v_world, np.array([-9.81], dtype=float)])

        if x_state.shape != (13,):
            raise RuntimeError(f"SRBD state shape mismatch: expected (13,), got {x_state.shape}")

        # Extract current foot positions relative to the base position (centroid approximation):
        #   r_i = p_foot_i_world - p_base_world
        pin.forwardKinematics(self._pin_model, self._pin_data, self._pin_q, self._pin_v)
        pin.updateFramePlacements(self._pin_model, self._pin_data)

        base_pos = self._pin_q[:3].copy()
        leg_order = ("lf", "lh", "rh", "rf")
        r_foot = np.zeros((4, 3), dtype=float)
        for leg_idx, leg in enumerate(leg_order):
            frame_id = self._foot_frame_ids[leg]
            r_foot[leg_idx, :] = np.array(self._pin_data.oMf[frame_id].translation, dtype=float) - base_pos

        # --- Build crawl contact schedule over MPC horizon ---
        # We mirror `GaitGenerator` stance logic so that we keep a stable support triangle.
        # phase offsets:
        #   lf=0.50, lh=0.75, rh=0.00, rf=0.25
        # stance if leg_phase >= (1-duty_factor)
        gait_cycle_time = float(max(self._gait_cycle_time_sec, 1e-6))
        duty_factor = float(max(min(self._gait_duty_factor, 0.999), 0.001))
        swing_phase_frac = float(1.0 - duty_factor)  # fraction of cycle spent in swing for each leg
        startup_elapsed_sec = (
            self.get_clock().now().nanoseconds - self._startup_time.nanoseconds
        ) * 1e-9
        cmd_idle = (
            abs(float(self._target_twist[0])) < float(self._idle_cmd_threshold)
            and abs(float(self._target_twist[1])) < float(self._idle_cmd_threshold)
            and abs(float(self._target_twist[2])) < float(self._idle_cmd_threshold)
        )

        max_joint_err, max_joint_vel = self._standing_tracking_metrics()
        if (not self._standup_complete) and (
            startup_elapsed_sec >= self._startup_full_stance_duration_sec
            and max_joint_err <= self._standup_joint_error_tolerance_rad
            and max_joint_vel <= self._standup_joint_velocity_tolerance_rad_s
        ):
            self._standup_complete = True
            self.get_logger().info(
                "Standup phase converged; enabling hold/locomotion transitions. "
                f"max_joint_err={max_joint_err:.3f}, max_joint_vel={max_joint_vel:.3f}"
            )

        control_mode = "mpc_gait"
        if not self._standup_complete:
            control_mode = "joint_pd_standup"
        elif cmd_idle:
            control_mode = "idle_hold"

        if control_mode == "mpc_gait" and not self._was_in_mpc_gait:
            # First tick of locomotion gait: reset phase counters and per-leg swing bookkeeping.
            self._gait_step_count = 0
            self._last_contact_now = np.array([1, 1, 0, 1], dtype=int)
            self._contact_blend = self._last_contact_now.astype(float).copy()
            self._swing_liftoff_step[:] = -1

        contact_pred = np.zeros((self.mpc_solver.N, 4), dtype=int)
        if control_mode != "mpc_gait":
            contact_pred[:, :] = 1
        else:
            for k in range(self.mpc_solver.N):
                step_k = self._gait_step_count + k
                t_k = float(step_k) * float(self.mpc_solver.dt)
                for leg_idx, leg in enumerate(self._leg_order):
                    phase_k_leg = (t_k / gait_cycle_time + float(self._crawl_phase_offsets[leg])) % 1.0
                    # stance if phase in [swing_phase_frac, 1)
                    contact_pred[k, leg_idx] = 1 if phase_k_leg >= swing_phase_frac else 0

        contact_now = contact_pred[0, :].astype(int)
        # 实测触地：刷新当前步的 contact_now（摆腿/混合仍用）；MPC 视界 contact_pred 仍跟步态生成。
        if self._use_gz_foot_contact:
            contact_now = self._gz_foot_in_contact.astype(int)
        self._update_contact_blend(contact_now)

        # Update swing targets on lift-off (stance -> swing)
        stance_duration_sec = float(duty_factor * gait_cycle_time)
        yaw_now = float(theta_rpy[2])
        vx_cmd, vy_cmd, _ = self._target_twist
        v_cmd_world_xy = self._cmd_vel_base_to_world_xy(yaw_now, vx_cmd, vy_cmd)
        v_actual_world_xyz = np.array([v_world[0], v_world[1], v_world[2]], dtype=float)
        omega_actual_world_xyz = np.array([omega_world[0], omega_world[1], omega_world[2]], dtype=float)

        # Low-pass filter the measured twist before Raibert to reject odom spikes.
        dt_twist = float(self._control_period_sec)
        tau_twist = float(max(self._twist_lpf_tau, 1e-4))
        alpha_twist = float(np.clip(dt_twist / (tau_twist + dt_twist), 0.0, 1.0))
        self._v_world_filt += alpha_twist * (v_actual_world_xyz - self._v_world_filt)
        self._w_world_filt += alpha_twist * (omega_actual_world_xyz - self._w_world_filt)

        base_height = float(max(p_world[2], self._min_base_height_for_raibert))
        delta_xy = np.zeros(2, dtype=float)
        if self._freeze_raibert_when_unstable:
            roll = float(theta_rpy[0])
            pitch = float(theta_rpy[1])
            unstable = (base_height < float(self._freeze_min_base_height)) or (
                abs(roll) > float(self._freeze_max_abs_rp_rad) or abs(pitch) > float(self._freeze_max_abs_rp_rad)
            )
            if not unstable:
                delta_xy = self._compute_raibert_delta_xy(
                    v_cmd_world_xy=v_cmd_world_xy,
                    v_actual_world_xyz=self._v_world_filt,
                    omega_actual_world_xyz=self._w_world_filt,
                    stance_duration=stance_duration_sec,
                    base_height=base_height,
                )
        else:
            delta_xy = self._compute_raibert_delta_xy(
                v_cmd_world_xy=v_cmd_world_xy,
                v_actual_world_xyz=self._v_world_filt,
                omega_actual_world_xyz=self._w_world_filt,
                stance_duration=stance_duration_sec,
                base_height=base_height,
            )

        for leg_idx in range(4):
            if int(self._last_contact_now[leg_idx]) == 1 and int(contact_now[leg_idx]) == 0:
                # Lift-off: store start and compute landing target (touchdown height = start height).
                self._swing_start_r[leg_idx, :] = r_foot[leg_idx, :].copy()
                self._swing_end_r[leg_idx, :] = r_foot[leg_idx, :].copy()
                self._swing_end_r[leg_idx, 0] += float(delta_xy[0])
                self._swing_end_r[leg_idx, 1] += float(delta_xy[1])
                self._swing_liftoff_step[leg_idx] = int(self._gait_step_count)

        # Initial condition: if we are at gait_step_count==0 and a leg is already swinging,
        # initialize its swing start/end (it won't pass through a "lift-off" event yet).
        if self._gait_step_count == 0:
            for leg_idx in range(4):
                if int(contact_now[leg_idx]) == 0:
                    self._swing_start_r[leg_idx, :] = r_foot[leg_idx, :].copy()
                    self._swing_end_r[leg_idx, :] = r_foot[leg_idx, :].copy()
                    self._swing_end_r[leg_idx, 0] += float(delta_xy[0])
                    self._swing_end_r[leg_idx, 1] += float(delta_xy[1])
                    self._swing_liftoff_step[leg_idx] = 0

        nominal_support = self.mpc_solver._build_nominal_support_u(contact_pred[0, :])[0, :]
        u_opt = nominal_support.copy()
        if control_mode == "mpc_gait":
            u_opt = self._solve_mpc_qp(x_state, r_foot, contact_pred)
            if u_opt.shape != (12,):
                raise RuntimeError(f"MPC control output shape mismatch: expected (12,), got {u_opt.shape}")

            # If MPC fell back (QP fail), add a conservative horizontal damping GRF for stance legs.
            if bool(getattr(self.mpc_solver, "last_used_fallback", False)):
                stance = (contact_now > 0.5).astype(float)
                n_stance = max(1.0, float(np.sum(stance)))

                # Total desired damping force in world XY.
                f_damp_xy = -self._fallback_kd_xy * v_world[:2]
                # Simple yaw-rate damping: create opposing lateral forces (very conservative).
                f_yaw_xy = -float(self._fallback_kd_omega_z) * float(omega_world[2]) * np.array([0.0, 1.0], dtype=float)
                f_total_xy = f_damp_xy + f_yaw_xy

                for leg_idx in range(4):
                    if not bool(stance[leg_idx] > 0.5):
                        continue
                    idx = 3 * leg_idx
                    fz = float(u_opt[idx + 2])
                    fz = float(np.clip(fz, self.mpc_solver.fz_min, self.mpc_solver.fz_max))
                    u_opt[idx + 2] = fz
                    max_xy = float(self.mpc_solver.mu) * fz

                    fx_add = float(f_total_xy[0] / n_stance)
                    fy_add = float(f_total_xy[1] / n_stance)
                    u_opt[idx + 0] = float(np.clip(float(u_opt[idx + 0]) + fx_add, -max_xy, max_xy))
                    u_opt[idx + 1] = float(np.clip(float(u_opt[idx + 1]) + fy_add, -max_xy, max_xy))

        # Whole-body mapping: tau_mpc = sum_i J_i^T * F_i, with J_i in R^(3x22), F_i in R^3.
        pin.computeJointJacobians(self._pin_model, self._pin_data, self._pin_q)
        pin.updateFramePlacements(self._pin_model, self._pin_data)

        tau_cmd = np.zeros(int(self._pin_model.nv), dtype=float)
        leg_order = ("lf", "lh", "rh", "rf")
        j_linear_by_leg: list[np.ndarray] = []
        for leg_idx, leg in enumerate(leg_order):
            frame_id = self._foot_frame_ids[leg]
            j6 = pin.getFrameJacobian(
                self._pin_model,
                self._pin_data,
                frame_id,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            )
            j_linear_by_leg.append(j6[:3, :])

        # --- Swing feet Cartesian PD (contact_pred[0]==0) ---
        # When a leg is in swing, MPC constraints force GRF to ~0 (especially Fz),
        # and we take over with a Cartesian impedance controller following a parabola.
        tau_pd_by_leg = [np.zeros(int(self._pin_model.nv), dtype=float) for _ in range(4)]
        swing_duration_sec = float(max(swing_phase_frac * gait_cycle_time, 1e-6))

        # Ensure frame velocity/placement are up-to-date for PD:
        pin.forwardKinematics(self._pin_model, self._pin_data, self._pin_q, self._pin_v)
        pin.updateFramePlacements(self._pin_model, self._pin_data)

        if control_mode == "mpc_gait":
            for leg_idx, leg in enumerate(leg_order):
                if int(contact_now[leg_idx]) != 0:
                    continue  # stance leg: no swing PD

                frame_id = self._foot_frame_ids[leg]

                r_start = self._swing_start_r[leg_idx, :]
                r_end = self._swing_end_r[leg_idx, :]

                liftoff_step = int(self._swing_liftoff_step[leg_idx])
                if liftoff_step < 0:
                    liftoff_step = int(self._gait_step_count)
                swing_elapsed_sec = float(self._gait_step_count - liftoff_step) * float(self.mpc_solver.dt)
                swing_alpha = float(np.clip(swing_elapsed_sec / swing_duration_sec, 0.0, 1.0))

                # Base-relative desired position (world axes).
                r_des = (1.0 - swing_alpha) * r_start + swing_alpha * r_end
                # Parabolic clearance on Z (world z component).
                swing_shape = 4.0 * swing_alpha * (1.0 - swing_alpha)
                r_des[2] = r_start[2] + float(self._swing_clearance) * swing_shape

                p_cur_world = np.array(self._pin_data.oMf[frame_id].translation, dtype=float)
                p_des_world = base_pos + r_des

                v_motion = pin.getFrameVelocity(
                    self._pin_model,
                    self._pin_data,
                    frame_id,
                    pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
                )
                v_cur_world = np.asarray(v_motion.linear, dtype=float)

                dp = p_des_world - p_cur_world
                # v_des = 0 for the swing tracking target.
                F_pd = self._Kp_swing * dp + self._Kd_swing * (-v_cur_world)
                F_pd = np.clip(F_pd, self._F_pd_min, self._F_pd_max)

                j6_pd = pin.getFrameJacobian(
                    self._pin_model,
                    self._pin_data,
                    frame_id,
                    pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
                )
                j_linear_pd = j6_pd[:3, :]
                tau_pd_by_leg[leg_idx] = j_linear_pd.T @ F_pd

        # Final command: per-leg mutual exclusion + cross-fade (prevents MPC/PD double actuation).
        if control_mode == "mpc_gait":
            for leg_idx in range(4):
                alpha = float(np.clip(float(self._contact_blend[leg_idx]), 0.0, 1.0))  # 0=swing, 1=stance
                j_linear = j_linear_by_leg[leg_idx]
                f_grf = u_opt[3 * leg_idx : 3 * leg_idx + 3]
                tau_leg_mpc = j_linear.T @ f_grf
                tau_leg_pd = tau_pd_by_leg[leg_idx]
                tau_cmd += alpha * tau_leg_mpc + (1.0 - alpha) * tau_leg_pd

        # Add gravity compensation last.
        gravity = pin.computeGeneralizedGravity(self._pin_model, self._pin_data, self._pin_q)
        if control_mode == "mpc_gait":
            tau_cmd = tau_cmd + gravity
        else:
            # joint_pd_standup / idle_hold：
            # - 对期望姿态做 ramp（只在 standup 阶段从当前关节角过渡到 standing_pose）
            # - 如果检测到足端仍在地面以下，则临时软化 kp、提高 kd
            hold_gravity = gravity if self._standup_use_gravity_compensation else np.zeros_like(gravity)
            ramp_alpha = 1.0
            if control_mode == "joint_pd_standup":
                ramp_alpha = float(np.clip(startup_elapsed_sec / float(self._standup_ramp_time_sec), 0.0, 1.0))
                if self._standup_q_start is None:
                    self._standup_q_start = self._pin_q.copy()

            base_pos = self._pin_q[:3].copy()
            min_foot_world_z = float(base_pos[2] + float(np.min(r_foot[:, 2])))
            tau_cmd = self._joint_space_hold_tau(
                gravity=hold_gravity,
                ramp_alpha=ramp_alpha,
                min_foot_world_z=min_foot_world_z,
            )

        # Rails are currently expected to stay locked near the standing pose.
        # Do not let whole-body force allocation drive the prismatic joints freely.
        for rail_joint_name in self._rail_joint_names:
            q_idx = self._pin_q_idx_by_motor[rail_joint_name]
            v_idx = self._pin_v_idx_by_motor[rail_joint_name]
            q_cur = float(self._pin_q[q_idx])
            v_cur = float(self._pin_v[v_idx])
            q_des = float(self._rail_hold_targets.get(rail_joint_name, 0.0))
            tau_cmd[v_idx] = (
                -float(self._rail_lock_kp) * (q_cur - q_des)
                - float(self._rail_lock_kd) * v_cur
            )

        # Pre-publish hard guardrails: NaN/Inf and absolute torque limiting.
        if not bool(np.all(np.isfinite(tau_cmd))):
            return
        tau_abs_limit = float(self._tau_abs_limit) if control_mode == "mpc_gait" else float(self._standup_tau_abs_limit)
        tau_cmd = np.clip(tau_cmd, -tau_abs_limit, tau_abs_limit)

        target_efforts: dict[str, float] = {}
        for motor_name in self._ordered_joint_names:
            v_idx = self._pin_v_idx_by_motor[motor_name]
            target_efforts[motor_name] = float(tau_cmd[v_idx])

        self._publish_efforts(target_efforts)

        # Advance gait phase (discrete time).
        self._last_contact_now = contact_now.copy()
        if control_mode == "mpc_gait":
            self._gait_step_count += 1
        self._was_in_mpc_gait = bool(control_mode == "mpc_gait")

        if self._debug_mode:
            vx, vy, omega = self._target_twist
            self.get_logger().debug(
                f"TODO MPC step. cmd_vel=(vx={vx:.3f}, vy={vy:.3f}, w={omega:.3f})"
            )

            now = time.time()
            if now - self._last_debug_log_walltime >= 2.0:
                self.get_logger().info(
                    "Publishing MPC effort commands "
                    f"(mode={control_mode}, odom='{self._odom_topic}', fz_leg={u_opt[2]:.2f}N)."
                )
                self._last_debug_log_walltime = now

        # Keep joints holding behavior controlled by Gazebo (hold_joints=true in gz_ros2_control)
        # until MPC writes non-zero efforts.


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MPCRobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.context.ok():
            node.get_logger().info("KeyboardInterrupt: shutting down MPC controller.")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
