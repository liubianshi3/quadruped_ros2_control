"""腿部运动学求解器（rail + 3R）。

dog2 单腿为 4-DoF 冗余构型：1 个 prismatic rail + 3 个旋转关节。
本实现采用“参数化降维”策略：rail 位移作为外部前置调度量，在固定 rail 下求解 3R 逆解。
"""

from typing import Optional, Tuple, Dict, Any
import logging
import numpy as np
from .leg_parameters import LegParameters
from .joint_semantics import JOINT_AXIS_LOCAL


class KinematicsSolver:
    """
    运动学求解器

    负责计算腿部的逆运动学和正运动学。
    当前阶段：导轨默认锁定在0.0米，但保留 rail_offset 接口。
    IK 使用与 FK 一致的几何链进行数值求解（阻尼最小二乘）。
    """

    def __init__(self, leg_params: Dict[str, LegParameters]):
        self.leg_params = leg_params
        self.logger = logging.getLogger(__name__)

        # 3-DoF 基线求解的正则化参数（可作为后续QP版本的先验）
        self._rail_candidates = 17
        self._dls_lambda = 2e-3
        self._max_iterations = 100
        self._position_tolerance = 5e-3
        self._rail_neutral_weight = 0.25
        self._posture_weight = 0.08
        self._smooth_weight = 0.06

        # FK 标定站立姿态（q = [haa, hfe, kfe]）
        self._standing_angles = {
            'lf': np.array([0.0, 0.3000, -0.5000]),
            'lh': np.array([0.0, 0.3000, -0.5000]),
            'rh': np.array([0.0, 0.3000, -0.5000]),
            'rf': np.array([0.0, 0.3000, -0.5000]),
        }

        # 每条腿最近一次有效解，用于连续性正则
        self._last_solution: Dict[str, Tuple[float, np.ndarray]] = {}

    def configure_regularization(self, cfg: Dict[str, Any]) -> None:
        """配置IK正则化参数（缺失字段保持现值）。"""
        self._rail_candidates = max(1, int(cfg.get('rail_candidates', self._rail_candidates)))
        self._dls_lambda = max(1e-9, float(cfg.get('dls_lambda', self._dls_lambda)))
        self._max_iterations = max(1, int(cfg.get('max_iterations', self._max_iterations)))
        self._position_tolerance = max(1e-6, float(cfg.get('position_tolerance', self._position_tolerance)))
        self._rail_neutral_weight = max(0.0, float(cfg.get('rail_neutral_weight', self._rail_neutral_weight)))
        self._posture_weight = max(0.0, float(cfg.get('posture_weight', self._posture_weight)))
        self._smooth_weight = max(0.0, float(cfg.get('smooth_weight', self._smooth_weight)))

        self.logger.info(
            "IK regularization updated: rails=%d dls=%.4g iter=%d tol=%.4g w_rail=%.4g w_posture=%.4g w_smooth=%.4g",
            self._rail_candidates,
            self._dls_lambda,
            self._max_iterations,
            self._position_tolerance,
            self._rail_neutral_weight,
            self._posture_weight,
            self._smooth_weight,
        )

    def _rotation_matrix_from_rpy(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        """从RPY角度创建旋转矩阵。"""
        Rx = np.array([
            [1.0, 0.0, 0.0],
            [0.0, np.cos(roll), -np.sin(roll)],
            [0.0, np.sin(roll), np.cos(roll)]
        ])
        Ry = np.array([
            [np.cos(pitch), 0.0, np.sin(pitch)],
            [0.0, 1.0, 0.0],
            [-np.sin(pitch), 0.0, np.cos(pitch)]
        ])
        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0.0],
            [np.sin(yaw), np.cos(yaw), 0.0],
            [0.0, 0.0, 1.0]
        ])
        return Rz @ Ry @ Rx

    def _transform_to_leg_frame(self, foot_pos: np.ndarray, leg_id: str) -> np.ndarray:
        """将脚部位置从base_link坐标系转换到腿部局部坐标系。"""
        params = self.leg_params[leg_id]
        pos_relative = foot_pos - params.base_position
        roll, pitch, yaw = params.base_rotation
        R = self._rotation_matrix_from_rpy(roll, pitch, yaw)
        return R.T @ pos_relative

    def _transform_from_leg_frame(self, pos_local: np.ndarray, leg_id: str) -> np.ndarray:
        """将位置从腿部局部坐标系转换到base_link坐标系。"""
        params = self.leg_params[leg_id]
        roll, pitch, yaw = params.base_rotation
        R = self._rotation_matrix_from_rpy(roll, pitch, yaw)
        return R @ pos_local + params.base_position

    def _homogeneous(self, R: np.ndarray, t: np.ndarray) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        return T

    def _rot_axis_angle(self, axis: np.ndarray, angle: float) -> np.ndarray:
        axis = axis / (np.linalg.norm(axis) + 1e-12)
        x, y, z = axis
        c = np.cos(angle)
        s = np.sin(angle)
        C = 1 - c
        return np.array([
            [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C]
        ])

    def _forward_local(self, params: LegParameters, s_m: float, q: np.ndarray) -> np.ndarray:
        """在腿局部坐标系中计算脚点位置。

        末端点定义为 URDF 中 foot_link 坐标系原点（tibia_link 经固定关节 foot_tip 偏移）：
        - 与 dog2.urdf.xacro 中 *_foot_fixed / *_foot_link 及 Pinocchio/MPC 足端帧一致
        """
        hip_roll_rad, hip_pitch_rad, knee_pitch_rad = q

        # rail semantic contract: +q translates along base_link-local +X.
        rail_translation_axis = JOINT_AXIS_LOCAL["rail"]
        T = self._homogeneous(np.eye(3), rail_translation_axis * float(s_m))

        # Revolute joint local axes stay shared across all four legs; only the
        # fixed origin transforms differ where mirror compensation is required.
        joint_axis = JOINT_AXIS_LOCAL["coxa"]
        T = T @ self._homogeneous(self._rotation_matrix_from_rpy(*params.hip_rpy), params.hip_offset)
        T = T @ self._homogeneous(self._rot_axis_angle(joint_axis, hip_roll_rad), np.zeros(3))

        # femur_joint origin and rotation
        T = T @ self._homogeneous(self._rotation_matrix_from_rpy(*params.knee_rpy), params.knee_offset)
        T = T @ self._homogeneous(self._rot_axis_angle(JOINT_AXIS_LOCAL["femur"], hip_pitch_rad), np.zeros(3))

        # tibia_joint origin and rotation
        T = T @ self._homogeneous(self._rotation_matrix_from_rpy(*params.tibia_rpy), params.tibia_offset)
        T = T @ self._homogeneous(self._rot_axis_angle(JOINT_AXIS_LOCAL["tibia"], knee_pitch_rad), np.zeros(3))
        T = T @ self._homogeneous(np.eye(3), params.foot_tip_offset_tibia)

        p = T @ np.array([0.0, 0.0, 0.0, 1.0])
        return p[:3]

    def _check_joint_limits(self, leg_id: str, hip_roll_rad: float, hip_pitch_rad: float, knee_pitch_rad: float) -> bool:
        params = self.leg_params[leg_id]
        limits = params.joint_limits

        coxa_min, coxa_max = limits['coxa']
        femur_min, femur_max = limits['femur']
        tibia_min, tibia_max = limits['tibia']

        if hip_roll_rad < coxa_min or hip_roll_rad > coxa_max:
            return False
        if hip_pitch_rad < femur_min or hip_pitch_rad > femur_max:
            return False
        if knee_pitch_rad < tibia_min or knee_pitch_rad > tibia_max:
            return False
        return True

    def _compute_local_jacobian(self, params: LegParameters, s_m: float, q: np.ndarray) -> np.ndarray:
        """数值计算3-DoF旋转关节的局部雅可比。"""
        cur = self._forward_local(params, s_m, q)
        finite_difference_step = 1e-6
        J = np.zeros((3, 3))
        for i in range(3):
            dq = np.zeros(3)
            dq[i] = finite_difference_step
            p2 = self._forward_local(params, s_m, q + dq)
            J[:, i] = (p2 - cur) / finite_difference_step
        return J

    def _build_initial_guesses(self, leg_id: str) -> Tuple[np.ndarray, ...]:
        """构建3R迭代初值集合，优先站立姿态和上一帧姿态。"""
        seeds = []
        if leg_id in self._last_solution:
            seeds.append(self._last_solution[leg_id][1].copy())

        seeds.append(self._standing_angles[leg_id].copy())
        seeds.extend([
            np.array([0.0, 0.7, -1.2]),
            np.array([0.0, 0.5, -1.0]),
            np.array([0.0, 1.0, -1.4]),
            np.array([0.3, 0.7, -1.0]),
            np.array([-0.3, 0.7, -1.0]),
            np.array([0.0, 1.3, -2.3]),
            np.array([0.0, 1.5, -2.5]),
            np.array([0.0, 1.6, -2.6]),
            np.array([0.3, 1.4, -2.4]),
            np.array([-0.3, 1.4, -2.4]),
            np.array([0.6, 1.5, -2.5]),
            np.array([-0.6, 1.5, -2.5]),
        ])
        return tuple(seeds)

    def _solve_3r_with_fixed_rail(
        self,
        leg_id: str,
        params: LegParameters,
        target_local: np.ndarray,
        s_m: float,
        q_ref: np.ndarray,
    ) -> Tuple[Optional[np.ndarray], float, Optional[np.ndarray]]:
        """在固定导轨位移下求解3R逆运动学（DLS + 正则）。"""
        rail_min, rail_max = params.joint_limits['rail']
        if s_m < rail_min or s_m > rail_max:
            return None, float('inf'), None

        best_q_local = None
        best_err_local = float('inf')
        best_seed_local = None

        for q0 in self._build_initial_guesses(leg_id):
            q = q0.copy()
            for _ in range(self._max_iterations):
                cur = self._forward_local(params, s_m, q)
                err_vec = target_local - cur
                err = float(np.linalg.norm(err_vec))
                if err < self._position_tolerance and self._check_joint_limits(leg_id, q[0], q[1], q[2]):
                    break

                J = self._compute_local_jacobian(params, s_m, q)
                JT = J.T
                H = JT @ J + self._dls_lambda * np.eye(3) + self._posture_weight * np.eye(3)
                g = JT @ err_vec + self._posture_weight * (q_ref - q)
                try:
                    dq = np.linalg.solve(H, g)
                except np.linalg.LinAlgError:
                    break

                dq = np.clip(dq, -0.2, 0.2)
                q = q + dq

                limits = params.joint_limits
                q[0] = np.clip(q[0], limits['coxa'][0], limits['coxa'][1])
                q[1] = np.clip(q[1], limits['femur'][0], limits['femur'][1])
                q[2] = np.clip(q[2], limits['tibia'][0], limits['tibia'][1])

            final_err = float(np.linalg.norm(target_local - self._forward_local(params, s_m, q)))
            if final_err < best_err_local and self._check_joint_limits(leg_id, q[0], q[1], q[2]):
                best_err_local = final_err
                best_q_local = q.copy()
                best_seed_local = q0.copy()

        return best_q_local, best_err_local, best_seed_local

    def _rail_regularization_cost(
        self,
        leg_id: str,
        params: LegParameters,
        s_m: float,
        q: np.ndarray,
        q_ref: np.ndarray,
    ) -> float:
        """导轨参数化代价：中位偏置 + 姿态偏差 + 连续性。"""
        rail_min, rail_max = params.joint_limits['rail']
        rail_mid = 0.5 * (rail_min + rail_max)
        rail_span = max(rail_max - rail_min, 1e-6)
        rail_norm = (s_m - rail_mid) / rail_span

        posture_err = q - q_ref
        cost = self._rail_neutral_weight * (rail_norm ** 2)
        cost += self._posture_weight * float(posture_err @ posture_err)

        if leg_id in self._last_solution:
            last_s, last_q = self._last_solution[leg_id]
            ds = (s_m - last_s) / rail_span
            dq = q - last_q
            cost += self._smooth_weight * (ds ** 2 + float(dq @ dq))

        return float(cost)

    def solve_ik(
        self,
        leg_id: str,
        foot_pos: Tuple[float, float, float],
        rail_offset: Optional[float] = None
    ) -> Optional[Tuple[float, float, float, float]]:
        """求解 IK（rail 参数化降维 + 固定 rail 的 3R-DLS）。

        Args:
            leg_id: lf/lh/rh/rf
            foot_pos: base_link 下脚端目标 (x, y, z)
            rail_offset: rail 位移调度量（m）。若为 None，则在 rail 范围内做离散搜索。

        Returns:
            (rail_m, hip_roll_rad, hip_pitch_rad, knee_pitch_rad) 或 None
        """
        params = self.leg_params[leg_id]
        rail_min, rail_max = params.joint_limits['rail']
        target_local = self._transform_to_leg_frame(np.array(foot_pos), leg_id)
        q_ref = self._standing_angles[leg_id]

        if not np.all(np.isfinite(target_local)):
            return None
        reach = float(params.link_lengths[1] + params.link_lengths[2])
        max_dist = reach + 1.0
        if float(np.linalg.norm(target_local)) > max_dist:
            return None

        if rail_offset is not None:
            requested_rail = float(rail_offset)
            rail_limit_tolerance = 1e-9
            if requested_rail < rail_min - rail_limit_tolerance or requested_rail > rail_max + rail_limit_tolerance:
                return None
            rail_candidates = [float(np.clip(requested_rail, rail_min, rail_max))]
        else:
            rail_candidates = np.linspace(rail_min, rail_max, self._rail_candidates).tolist()
            if leg_id in self._last_solution:
                rail_candidates.append(float(np.clip(self._last_solution[leg_id][0], rail_min, rail_max)))

        best_q = None
        best_err = float('inf')
        best_seed = None
        best_s = 0.0
        best_total_cost = float('inf')

        for s in rail_candidates:
            q_candidate, err_candidate, seed_candidate = self._solve_3r_with_fixed_rail(
                leg_id=leg_id,
                params=params,
                target_local=target_local,
                s_m=float(s),
                q_ref=q_ref,
            )
            if q_candidate is None:
                continue

            reg_cost = self._rail_regularization_cost(leg_id, params, float(s), q_candidate, q_ref)
            total_cost = err_candidate + reg_cost
            if total_cost < best_total_cost:
                best_total_cost = total_cost
                best_q = q_candidate
                best_err = err_candidate
                best_seed = seed_candidate
                best_s = float(s)

        if best_q is not None and best_err < self._position_tolerance:
            self._last_solution[leg_id] = (best_s, best_q.copy())
            return (best_s, float(best_q[0]), float(best_q[1]), float(best_q[2]))

        self.logger.warning(
            "IK failed leg=%s rail=%.4f target_local=%s best_err=%.6f best_q=%s best_seed=%s",
            leg_id,
            best_s,
            np.array2string(target_local, precision=5, suppress_small=True),
            best_err,
            None if best_q is None else np.array2string(best_q, precision=5, suppress_small=True),
            None if best_seed is None else np.array2string(best_seed, precision=5, suppress_small=True),
        )

        return None

    def solve_fk(
        self,
        leg_id: str,
        joint_positions: Tuple[float, float, float, float]
    ) -> Tuple[float, float, float]:
        """求解正运动学。"""
        params = self.leg_params[leg_id]
        rail_m, hip_roll_rad, hip_pitch_rad, knee_pitch_rad = joint_positions

        pos_local = self._forward_local(
            params,
            rail_m,
            np.array([hip_roll_rad, hip_pitch_rad, knee_pitch_rad])
        )
        pos_global = self._transform_from_leg_frame(pos_local, leg_id)
        return (float(pos_global[0]), float(pos_global[1]), float(pos_global[2]))


def create_kinematics_solver(model_variant: str = "real") -> KinematicsSolver:
    from .model_variant import get_leg_parameters
    return KinematicsSolver(get_leg_parameters(model_variant))


def create_symmetric_kinematics_solver() -> KinematicsSolver:
    return create_kinematics_solver(model_variant="symmetric")


if __name__ == '__main__':
    raise SystemExit(
        "kinematics_solver.py is a library module; run package tests for validation."
    )
