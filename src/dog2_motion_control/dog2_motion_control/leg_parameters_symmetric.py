"""
dog2_symmetric 腿部参数数据模型

配套 dog2_symmetric.urdf.xacro 的独立控制参数。
与 leg_parameters.py 完全独立，互不覆盖。
对称化变更：
  - leg mount 严格对称 (lf X=-rf X, lh X=-rh X, Y 完全镜像)
  - leg4 hip_offset = 0.016（镜像自 lh）
  - leg3/leg4 knee_offset z = -0.0274（与左侧 knee magnitude 一致）
  - leg4 shin_xyz = leg3 shin_xyz（无独立漂移）
  - foot4 = foot3.copy()
  - shin x: 左侧 +0.026, 右侧 -0.026
"""

from dataclasses import dataclass
from typing import Dict, Tuple
import numpy as np

from .joint_semantics import (
    COXA_ORIGIN_RPY,
    FEMUR_ORIGIN_RPY,
    LEG_BASE_RPY,
    TIBIA_ORIGIN_RPY,
)
from .urdf_joint_limits import (
    clear_dog2_urdf_joint_limits_cache,
    load_dog2_urdf_joint_limits,
)


@dataclass
class LegParameters:
    """
    单条腿的参数配置

    Attributes:
        leg_id: 腿部标识符 ('lf', 'rf', 'lh', 'rh')
        leg_num: 腿部编号 (1, 2, 3, 4)
        base_position: 腿部基座在 base_link 中的 rail 锚点 [x, y, z] (米)；与 URDF *_rail_joint origin 一致
        base_rotation: 腿部坐标系旋转 [roll, pitch, yaw] (弧度)
        hip_offset:  rail_link -> coxa_joint 的位移 [x, y, z] (米)
        hip_rpy: coxa_joint origin 的固定 rpy [roll, pitch, yaw] (弧度)
        knee_offset: coxa_link -> femur_joint 的位移 [x, y, z] (米)
        knee_rpy: femur_joint origin 的固定 rpy [roll, pitch, yaw] (弧度)
        tibia_offset: femur_link -> tibia_joint 的位移 [x, y, z] (米)
        tibia_rpy: tibia_joint origin 的固定 rpy [roll, pitch, yaw] (弧度)
        link_lengths: 链长 (HFE到KFE, KFE到foot) (米)
        joint_limits: 关节限位字典 {'rail': (下限, 上限), 'coxa': (下限, 上限), ...}
        foot_tip_offset_tibia: foot_link 原点在 tibia_link 坐标系下的平移（与 URDF *_foot_fixed 一致）
        rail_locked: 导轨是否锁定（当前阶段为True）
    """
    leg_id: str
    leg_num: int
    base_position: np.ndarray
    base_rotation: np.ndarray
    hip_offset: np.ndarray
    hip_rpy: np.ndarray
    knee_offset: np.ndarray
    knee_rpy: np.ndarray
    tibia_offset: np.ndarray
    tibia_rpy: np.ndarray
    link_lengths: Tuple[float, float, float]
    joint_limits: Dict[str, Tuple[float, float]]
    shin_xyz: np.ndarray = None
    foot_tip_offset_tibia: np.ndarray = None
    rail_locked: bool = True

    def __post_init__(self):
        """验证参数有效性"""
        if self.leg_id not in ['lf', 'rf', 'lh', 'rh']:
            raise ValueError(f"Invalid leg_id: {self.leg_id}")
        if self.leg_num not in [1, 2, 3, 4]:
            raise ValueError(f"Invalid leg_num: {self.leg_num}")
        if len(self.link_lengths) != 3:
            raise ValueError(f"link_lengths must have 3 elements, got {len(self.link_lengths)}")
        if self.hip_offset.shape != (3,) or self.knee_offset.shape != (3,) or self.tibia_offset.shape != (3,):
            raise ValueError("hip_offset/knee_offset/tibia_offset must be 3D vectors")
        if self.hip_rpy.shape != (3,) or self.knee_rpy.shape != (3,) or self.tibia_rpy.shape != (3,):
            raise ValueError("hip_rpy/knee_rpy/tibia_rpy must be 3D vectors")
        if self.shin_xyz is None:
            raise ValueError("shin_xyz must be provided (tibia_link inertial origin offset)")
        if np.asarray(self.shin_xyz).shape != (3,):
            raise ValueError("shin_xyz must be a 3D vector")
        if self.foot_tip_offset_tibia is None:
            raise ValueError("foot_tip_offset_tibia must be provided (URDF foot_tip_xyz in tibia_link frame)")
        if np.asarray(self.foot_tip_offset_tibia).shape != (3,):
            raise ValueError("foot_tip_offset_tibia must be a 3D vector")


def create_leg_parameters_symmetric() -> Dict[str, LegParameters]:
    """
    创建所有4条腿的完全对称参数配置

    对应 dog2_symmetric.urdf.xacro 的对称化设计

    Returns:
        字典，键为leg_id ('lf', 'rf', 'lh', 'rh')，值为LegParameters对象
    """
    L1 = 0.055
    L2 = 0.15201
    L3 = 0.299

    link_lengths = (L1, L2, L3)

    FOOT_TIP_LATERAL_INBOARD_M = 0.024
    FOOT_SPHERE_RADIUS_M = 0.012
    FOOT_TIP_PLUS_Y_HALF_DIAMETER_M = FOOT_SPHERE_RADIUS_M
    FOOT_TIP_MINUS_Y_ONE_AND_HALF_DIAMETER_M = 3.0 * FOOT_SPHERE_RADIUS_M
    FOOT_TIP_Z_DOWN_M = (
        2.0 * 0.75 * 2.0 * FOOT_SPHERE_RADIUS_M
        + 2.0 * (2.0 * FOOT_SPHERE_RADIUS_M)
        - 3.0 * (2.0 * FOOT_SPHERE_RADIUS_M)
    )

    def _foot_tip_yz(sy: float, sz: float, length: float) -> np.ndarray:
        n = float(np.linalg.norm(np.array([sy, sz], dtype=float)))
        return np.array([0.0, sy / n * length, sz / n * length], dtype=float)

    foot12 = _foot_tip_yz(-0.143524743603395, -0.0694046953395906, L3)
    foot12[0] = FOOT_TIP_LATERAL_INBOARD_M
    foot12[1] += FOOT_TIP_PLUS_Y_HALF_DIAMETER_M - FOOT_TIP_MINUS_Y_ONE_AND_HALF_DIAMETER_M
    foot12[2] -= FOOT_TIP_Z_DOWN_M

    foot3 = _foot_tip_yz(-0.143524743603395, -0.0694046953395908, L3)
    foot3[0] = -FOOT_TIP_LATERAL_INBOARD_M
    foot3[1] += FOOT_TIP_PLUS_Y_HALF_DIAMETER_M - FOOT_TIP_MINUS_Y_ONE_AND_HALF_DIAMETER_M
    foot3[2] -= FOOT_TIP_Z_DOWN_M

    # symmetric: foot4 = foot3
    foot4 = foot3.copy()

    urdf_joint_limits = load_dog2_urdf_joint_limits()
    joint_limits_template = {
        role: tuple(limits)
        for role, limits in urdf_joint_limits.revolute_by_role.items()
    }

    # leg1: lf, symmetric base_position (absolute CAD 0.12685,0.0625)
    leg1_params = LegParameters(
        leg_id='lf',
        leg_num=1,
        base_position=np.array([0.12685, 0.0625, 0.0]),
        base_rotation=LEG_BASE_RPY.copy(),
        hip_offset=np.array([-0.016, 0.0199, 0.055]),
        hip_rpy=COXA_ORIGIN_RPY.copy(),
        knee_offset=np.array([-0.0233, -0.055, 0.0274]),
        knee_rpy=FEMUR_ORIGIN_RPY.copy(),
        tibia_offset=np.array([0.0, -0.15201, 0.12997]),
        tibia_rpy=TIBIA_ORIGIN_RPY.copy(),
        link_lengths=link_lengths,
        joint_limits={
            'rail': urdf_joint_limits.rail_by_leg['lf'],
            **joint_limits_template
        },
        shin_xyz=np.array([0.026, -0.1435, -0.0694]),
        foot_tip_offset_tibia=foot12.copy(),
        rail_locked=True
    )

    # leg2: lh, symmetric base_position (absolute CAD 0.3711,0.0625)
    leg2_params = LegParameters(
        leg_id='lh',
        leg_num=2,
        base_position=np.array([0.3711, 0.0625, 0.0]),
        base_rotation=LEG_BASE_RPY.copy(),
        hip_offset=np.array([0.016, 0.0199, 0.055]),
        hip_rpy=COXA_ORIGIN_RPY.copy(),
        knee_offset=np.array([-0.0233, -0.055, 0.0274]),
        knee_rpy=FEMUR_ORIGIN_RPY.copy(),
        tibia_offset=np.array([0.0, -0.15201, 0.12997]),
        tibia_rpy=TIBIA_ORIGIN_RPY.copy(),
        link_lengths=link_lengths,
        joint_limits={
            'rail': urdf_joint_limits.rail_by_leg['lh'],
            **joint_limits_template
        },
        shin_xyz=np.array([0.026, -0.1435, -0.0694]),
        foot_tip_offset_tibia=foot12.copy(),
        rail_locked=True
    )

    # leg3: rh, symmetric base_position (absolute CAD 0.3711,0.1825), knee_z = -0.0274
    leg3_params = LegParameters(
        leg_id='rh',
        leg_num=3,
        base_position=np.array([0.3711, 0.1825, 0.0]),
        base_rotation=LEG_BASE_RPY.copy(),
        hip_offset=np.array([-0.016, 0.0199, 0.055]),
        hip_rpy=COXA_ORIGIN_RPY.copy(),
        knee_offset=np.array([-0.0233, -0.055, -0.0274]),
        knee_rpy=FEMUR_ORIGIN_RPY.copy(),
        tibia_offset=np.array([0.0, -0.15201, 0.12997]),
        tibia_rpy=TIBIA_ORIGIN_RPY.copy(),
        link_lengths=link_lengths,
        joint_limits={
            'rail': urdf_joint_limits.rail_by_leg['rh'],
            **joint_limits_template
        },
        shin_xyz=np.array([-0.026, -0.1435, -0.0694]),
        foot_tip_offset_tibia=foot3.copy(),
        rail_locked=True
    )

    # leg4: rf, symmetric base_position (absolute CAD 0.12685,0.1825),
    # hip_offset mirror of lh, knee_z = -0.0274, foot = foot3
    leg4_params = LegParameters(
        leg_id='rf',
        leg_num=4,
        base_position=np.array([0.12685, 0.1825, 0.0]),
        base_rotation=LEG_BASE_RPY.copy(),
        hip_offset=np.array([0.016, 0.0199, 0.055]),
        hip_rpy=COXA_ORIGIN_RPY.copy(),
        knee_offset=np.array([-0.0233, -0.055, -0.0274]),
        knee_rpy=FEMUR_ORIGIN_RPY.copy(),
        tibia_offset=np.array([0.0, -0.15201, 0.12997]),
        tibia_rpy=TIBIA_ORIGIN_RPY.copy(),
        link_lengths=link_lengths,
        joint_limits={
            'rail': urdf_joint_limits.rail_by_leg['rf'],
            **joint_limits_template
        },
        shin_xyz=np.array([-0.026, -0.1435, -0.0694]),
        foot_tip_offset_tibia=foot4.copy(),
        rail_locked=True
    )

    return {
        'lf': leg1_params,
        'lh': leg2_params,
        'rh': leg3_params,
        'rf': leg4_params,
    }


def reload_leg_parameter_joint_limits_from_urdf(force_reload: bool = False) -> str:
    """Refresh shared LegParameters limits from the authoritative xacro source."""

    if force_reload:
        clear_dog2_urdf_joint_limits_cache()

    urdf_joint_limits = load_dog2_urdf_joint_limits()
    for leg_id, params in LEG_PARAMETERS_SYMMETRIC.items():
        params.joint_limits["rail"] = urdf_joint_limits.rail_by_leg[leg_id]
        for role, limits in urdf_joint_limits.revolute_by_role.items():
            params.joint_limits[role] = limits
    return urdf_joint_limits.source_path


# 预创建的全局参数对象（symmetric 版本）
LEG_PARAMETERS_SYMMETRIC = create_leg_parameters_symmetric()


def get_leg_parameters_symmetric(leg_id: str) -> LegParameters:
    """
    获取指定腿部的 symmetric 参数

    Args:
        leg_id: 腿部标识符 ('lf', 'rf', 'lh', 'rh')

    Returns:
        LegParameters对象

    Raises:
        ValueError: 如果leg_id无效
    """
    if leg_id not in LEG_PARAMETERS_SYMMETRIC:
        raise ValueError(f"Invalid leg_id: {leg_id}. Must be one of {list(LEG_PARAMETERS_SYMMETRIC.keys())}")
    return LEG_PARAMETERS_SYMMETRIC[leg_id]


if __name__ == '__main__':
    print("Symmetric 腿部参数配置：\n")
    for leg_id, params in LEG_PARAMETERS_SYMMETRIC.items():
        print(f"{leg_id} (leg{params.leg_num}):")
        print(f"  基座位置: {params.base_position}")
        print(f"  基座旋转 (rpy): {params.base_rotation}")
        print(f"  链长: {params.link_lengths}")
        print(f"  导轨限位: {params.joint_limits['rail']}")
        print(f"  导轨锁定: {params.rail_locked}")
        print()
