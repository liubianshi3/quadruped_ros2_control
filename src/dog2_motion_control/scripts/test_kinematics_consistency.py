#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Offline regression tests for dog2 leg kinematics.

目标：
- 验证 KinematicsSolver 的 FK/IK 在 rail + 3R 空间内自洽；
- 验证 KinematicsSolver 的足端点与 Pinocchio 中 foot_link frame 原点对齐。

运行方式（在已构建的 ROS2 工作区环境中）：
    cd /home/dell/aperfect/carbot_ws
    python3 src/dog2_motion_control/scripts/test_kinematics_consistency.py
"""

from __future__ import annotations

import math
import os
import random
import sys
from dataclasses import dataclass
from typing import Dict, Tuple, Optional

import numpy as np

from ament_index_python.packages import get_package_share_directory

def _ensure_import_path() -> None:
    """允许在未 `colcon build` / 未 source 环境时直接运行脚本。"""
    this_file = os.path.abspath(__file__)
    # .../src/dog2_motion_control/scripts/test_kinematics_consistency.py
    pkg_root = os.path.dirname(os.path.dirname(this_file))  # .../src/dog2_motion_control
    if pkg_root not in sys.path:
        sys.path.insert(0, pkg_root)


_ensure_import_path()

from dog2_motion_control.kinematics_solver import create_kinematics_solver  # noqa: E402
from dog2_motion_control.leg_parameters import LEG_PARAMETERS  # noqa: E402


@dataclass
class Thresholds:
    # FK->IK->FK roundtrip
    roundtrip_rail_pass_m: float = 1e-8
    roundtrip_rail_warn_m: float = 1e-5
    roundtrip_joint_pass_rad: float = 1e-5
    roundtrip_joint_warn_rad: float = 5e-3
    roundtrip_foot_pass_m: float = 5e-4
    roundtrip_foot_warn_m: float = 5e-3

    # FK foot vs pinocchio foot_link
    align_foot_pass_m: float = 5e-6
    align_foot_warn_m: float = 2e-3


def _grade(value: float, pass_th: float, warn_th: float) -> str:
    if value <= pass_th:
        return "PASS"
    if value <= warn_th:
        return "WARN"
    return "FAIL"


def _sample_joint_state(
    leg_id: str,
    *,
    solver=None,
    mode: str = "near_standing",
) -> Tuple[float, float, float, float]:
    """采样一组 (rail, coxa, femur, tibia)。

    - near_standing: 在 solver 的 standing 姿态附近做小扰动（更贴近在线控制分布）
    - full_limits: 全限位随机（更严格，但不保证数值 IK 必收敛）
    """
    params = LEG_PARAMETERS[leg_id]
    rail_min, rail_max = params.joint_limits["rail"]
    coxa_min, coxa_max = params.joint_limits["coxa"]
    femur_min, femur_max = params.joint_limits["femur"]
    tibia_min, tibia_max = params.joint_limits["tibia"]

    if mode == "full_limits":
        rail = random.uniform(rail_min, rail_max)
        coxa = random.uniform(coxa_min, coxa_max)
        femur = random.uniform(femur_min, femur_max)
        tibia = random.uniform(tibia_min, tibia_max)
        return rail, coxa, femur, tibia

    # 默认：在站立姿态附近采样，避免“无 seed 的数值 IK”在全空间随机姿态下误报失败
    if solver is None:
        solver = create_kinematics_solver()
    q_ref = np.asarray(solver._standing_angles[leg_id], dtype=float)  # (coxa, femur, tibia)

    # 站立附近扰动幅度（弧度）
    d_coxa = random.uniform(-0.5, 0.5)
    d_femur = random.uniform(-0.7, 0.7)
    d_tibia = random.uniform(-0.7, 0.7)
    coxa = float(np.clip(q_ref[0] + d_coxa, coxa_min, coxa_max))
    femur = float(np.clip(q_ref[1] + d_femur, femur_min, femur_max))
    tibia = float(np.clip(q_ref[2] + d_tibia, tibia_min, tibia_max))

    # rail 也在中间附近采样，避免极限位导致某些姿态数值条件很差
    rail_mid = 0.5 * (rail_min + rail_max)
    rail_span = (rail_max - rail_min)
    rail = float(np.clip(rail_mid + random.uniform(-0.35, 0.35) * rail_span, rail_min, rail_max))
    return rail, coxa, femur, tibia


def test_fk_ik_roundtrip(num_samples_per_leg: int = 50, th: Optional[Thresholds] = None) -> None:
    """对每条腿做若干次一致性回归：

    - 几何自洽（确定性）：FK(global)->to_leg_frame 与 _forward_local(local) 是否一致
    - IK roundtrip（统计性）：在 rail 固定条件下，solve_ik 是否能回到同一 foot（不强求回到同一关节）
    """
    th = th or Thresholds()
    solver = create_kinematics_solver()
    # 静音 IK 内部 warning，避免海量失败日志淹没回归输出
    try:
        import logging

        solver.logger.setLevel(logging.ERROR)
        solver.logger.propagate = False
    except Exception:
        pass
    leg_ids = ["lf", "lh", "rh", "rf"]

    per_leg_max = {
        leg_id: {"rail": 0.0, "joint": 0.0, "foot": 0.0, "geom_local": 0.0} for leg_id in leg_ids
    }
    worst_overall = {
        "leg": None,
        "q": None,
        "ik": None,
        "foot": None,
        "foot2": None,
        "rail_err": -1.0,
        "joint_err": -1.0,
        "foot_err": -1.0,
    }
    failures = {leg_id: 0 for leg_id in leg_ids}
    worst_failure = {"leg": None, "q": None, "foot": None}

    for leg_id in leg_ids:
        for _ in range(num_samples_per_leg):
            q = _sample_joint_state(leg_id, solver=solver, mode="near_standing")
            foot = solver.solve_fk(leg_id, q)

            # --- 几何自洽（不依赖 IK 数值收敛） ---
            params = solver.leg_params[leg_id]
            local_from_global = solver._transform_to_leg_frame(np.array(foot, dtype=float), leg_id)
            local_fk = solver._forward_local(
                params,
                float(q[0]),
                np.array([float(q[1]), float(q[2]), float(q[3])], dtype=float),
            )
            geom_err = float(np.linalg.norm(np.asarray(local_from_global) - np.asarray(local_fk)))
            per_leg_max[leg_id]["geom_local"] = max(per_leg_max[leg_id]["geom_local"], geom_err)

            ik = solver.solve_ik(leg_id, foot, rail_offset=q[0])
            if ik is None:
                failures[leg_id] += 1
                worst_failure = {"leg": leg_id, "q": q, "foot": foot}
                continue

            rail2, coxa2, femur2, tibia2 = ik
            rail1, coxa1, femur1, tibia1 = q

            # 数值上 rail 可能存在极小误差，这里仍然以原 rail 为真值。
            rail_err = abs(rail2 - rail1)
            joint_err = max(
                abs(coxa2 - coxa1),
                abs(femur2 - femur1),
                abs(tibia2 - tibia1),
            )

            foot2 = solver.solve_fk(leg_id, ik)
            pos_err = float(np.linalg.norm(np.asarray(foot2) - np.asarray(foot)))
            per_leg_max[leg_id]["rail"] = max(per_leg_max[leg_id]["rail"], rail_err)
            per_leg_max[leg_id]["joint"] = max(per_leg_max[leg_id]["joint"], joint_err)
            per_leg_max[leg_id]["foot"] = max(per_leg_max[leg_id]["foot"], pos_err)

            # 以 foot_err 为主导，rail/joint 为辅，记录最坏样本以便复现
            if (
                pos_err > float(worst_overall["foot_err"])
                or (math.isclose(pos_err, float(worst_overall["foot_err"])) and rail_err > float(worst_overall["rail_err"]))
            ):
                worst_overall.update(
                    {
                        "leg": leg_id,
                        "q": q,
                        "ik": ik,
                        "foot": foot,
                        "foot2": foot2,
                        "rail_err": rail_err,
                        "joint_err": joint_err,
                        "foot_err": pos_err,
                    }
                )

    print("=== FK/IK roundtrip consistency ===")
    for leg_id in leg_ids:
        g = per_leg_max[leg_id]["geom_local"]
        r = per_leg_max[leg_id]["rail"]
        j = per_leg_max[leg_id]["joint"]
        p = per_leg_max[leg_id]["foot"]
        print(
            f"{leg_id}: "
            f"geom_local_max={g:.6e}m[{_grade(g, 1e-8, 1e-5)}], "
            f"rail_max={r:.6e}m[{_grade(r, th.roundtrip_rail_pass_m, th.roundtrip_rail_warn_m)}], "
            f"joint_max={j:.6e}rad[{_grade(j, th.roundtrip_joint_pass_rad, th.roundtrip_joint_warn_rad)}], "
            f"foot_max={p:.6e}m[{_grade(p, th.roundtrip_foot_pass_m, th.roundtrip_foot_warn_m)}]"
        )

    geom_max = max(per_leg_max[l]["geom_local"] for l in leg_ids)
    rail_max = max(per_leg_max[l]["rail"] for l in leg_ids)
    joint_max = max(per_leg_max[l]["joint"] for l in leg_ids)
    foot_max = max(per_leg_max[l]["foot"] for l in leg_ids)
    print(
        "overall: "
        f"geom_local_max={geom_max:.6e}m[{_grade(geom_max, 1e-8, 1e-5)}], "
        f"rail_max={rail_max:.6e}m[{_grade(rail_max, th.roundtrip_rail_pass_m, th.roundtrip_rail_warn_m)}], "
        f"joint_max={joint_max:.6e}rad[{_grade(joint_max, th.roundtrip_joint_pass_rad, th.roundtrip_joint_warn_rad)}], "
        f"foot_max={foot_max:.6e}m[{_grade(foot_max, th.roundtrip_foot_pass_m, th.roundtrip_foot_warn_m)}]"
    )

    if worst_overall["leg"] is not None:
        print("\nworst-case sample (roundtrip):")
        print(f"leg={worst_overall['leg']}")
        print(f"q_in = {worst_overall['q']}")
        print(f"foot_fk(q_in) = {worst_overall['foot']}")
        print(f"ik(rail_fixed)= {worst_overall['ik']}")
        print(f"foot_fk(ik)   = {worst_overall['foot2']}")
        print(
            "errors: "
            f"rail={float(worst_overall['rail_err']):.6e}m, "
            f"joint={float(worst_overall['joint_err']):.6e}rad, "
            f"foot={float(worst_overall['foot_err']):.6e}m"
        )

    total_fail = sum(failures.values())
    total = int(num_samples_per_leg) * len(leg_ids)
    if total_fail > 0:
        print("\n[WARN] IK failures occurred in roundtrip sampling near standing.")
        for leg_id in leg_ids:
            if failures[leg_id]:
                print(f"{leg_id}: failures={failures[leg_id]}/{num_samples_per_leg}")
        if worst_failure["leg"] is not None:
            print("worst failure sample:")
            print(f"leg={worst_failure['leg']}")
            print(f"q_in={worst_failure['q']}")
            print(f"foot_fk(q_in)={worst_failure['foot']}")
    else:
        print("\nIK failures: 0 (near-standing sampling)")


def _build_pinocchio_model():
    """构建与 MPCRobotController 相同的 Pinocchio 模型。"""
    import pinocchio as pin
    import xacro

    # 优先走 ament_index；若当前 shell 未 source 工作区，则回退到工作区相对路径
    try:
        dog2_desc_share = get_package_share_directory("dog2_description")
        dog2_ctrl_share = get_package_share_directory("dog2_motion_control")
        urdf_xacro_path = os.path.join(dog2_desc_share, "urdf", "dog2.urdf.xacro")
        controllers_yaml_path = os.path.join(dog2_ctrl_share, "config", "effort_controllers.yaml")
    except Exception:
        ws_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
        urdf_xacro_path = os.path.join(ws_root, "src", "dog2_description", "urdf", "dog2.urdf.xacro")
        controllers_yaml_path = os.path.join(ws_root, "src", "dog2_motion_control", "config", "effort_controllers.yaml")
        if not os.path.exists(urdf_xacro_path):
            raise FileNotFoundError(f"URDF xacro not found at '{urdf_xacro_path}' (ament_index unavailable)")
        if not os.path.exists(controllers_yaml_path):
            # 控制器 yaml 仅用于 xacro mappings（有些 xacro 不实际读取该文件）
            controllers_yaml_path = ""

    doc = xacro.process_file(
        urdf_xacro_path,
        mappings={
            "controllers_yaml": controllers_yaml_path,
            "mass_scale": "1.0",
            "control_mode": "effort",
        },
    )
    model = pin.buildModelFromXML(doc.toxml(), pin.JointModelFreeFlyer())
    data = model.createData()
    return model, data


def _make_q_from_leg_state(
    model,
    leg_joint_state: Dict[str, Tuple[float, float, float, float]],
) -> np.ndarray:
    """构造 pinocchio q 向量（简单版本，只关心关节位置，不关心基座姿态）。"""
    import pinocchio as pin

    nq = int(model.nq)
    q = np.zeros(nq, dtype=float)

    # Free-flyer 设为单位姿态、略高的 z，避免生成退化 Jacobian。
    q[:7] = np.array([0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0], dtype=float)

    for leg_id, (rail, coxa, femur, tibia) in leg_joint_state.items():
        prefix = f"{leg_id}_"
        for joint_name, value in (
            (prefix + "rail_joint", rail),
            (prefix + "coxa_joint", coxa),
            (prefix + "femur_joint", femur),
            (prefix + "tibia_joint", tibia),
        ):
            jid = int(model.getJointId(joint_name))
            if jid == 0:
                raise RuntimeError(f"Joint '{joint_name}' not found in pinocchio model.")
            joint = model.joints[jid]
            if isinstance(joint, pin.JointModelFreeFlyer):
                raise RuntimeError(f"Unexpected freeflyer for motor joint '{joint_name}'")
            idx_q = int(joint.idx_q)
            if idx_q < 7 or idx_q >= nq:
                raise RuntimeError(f"Motor joint '{joint_name}' has invalid idx_q={idx_q} for nq={nq}")
            q[idx_q] = float(value)

    return q


def test_foot_frame_alignment(num_samples: int = 20, th: Optional[Thresholds] = None) -> None:
    """验证 IK/FK 足端点是否与 pinocchio foot_link frame 对齐。"""
    th = th or Thresholds()
    import pinocchio as pin

    model, data = _build_pinocchio_model()
    solver = create_kinematics_solver()
    try:
        import logging

        solver.logger.setLevel(logging.ERROR)
        solver.logger.propagate = False
    except Exception:
        pass

    # 针对当前实现，期望 nq=23（7 + 16 motors）
    if int(model.nq) != 23:
        print(f"[WARN] Expected pinocchio nq=23, got {model.nq}. Test may not be meaningful.")

    leg_ids = ["lf", "lh", "rh", "rf"]
    foot_frame_names = {
        "lf": "lf_foot_link",
        "lh": "lh_foot_link",
        "rh": "rh_foot_link",
        "rf": "rf_foot_link",
    }
    foot_frame_ids = {leg: int(model.getFrameId(name)) for leg, name in foot_frame_names.items()}
    # The leg chain attaches directly to the semantic body frame `base_link`.
    trunk_frame_name = "base_link"
    trunk_frame_id = int(model.getFrameId(trunk_frame_name))
    if trunk_frame_id >= int(model.nframes):
        raise RuntimeError(f"{trunk_frame_name} frame not found in pinocchio model.")

    per_leg_max = {leg_id: 0.0 for leg_id in leg_ids}
    worst = {"leg": None, "q_leg": None, "fk": None, "pin": None, "err": -1.0}

    for _ in range(num_samples):
        # 所有腿各采一组姿态，组成全身关节状态
        leg_qs: Dict[str, Tuple[float, float, float, float]] = {
            leg_id: _sample_joint_state(leg_id) for leg_id in leg_ids
        }
        q = _make_q_from_leg_state(model, leg_qs)
        # 更新 Pinocchio 模型
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        oMtrunk = data.oMf[trunk_frame_id]

        # 比较每条腿：FK 足端点 vs pinocchio frame
        for leg_id in leg_ids:
            fk_pos = np.asarray(solver.solve_fk(leg_id, leg_qs[leg_id]), dtype=float)
            frame_id = foot_frame_ids[leg_id]
            oMf = data.oMf[frame_id]
            pin_foot_world = np.asarray(oMf.translation, dtype=float)
            # 对齐到 merged trunk frame：p_trunk = (oMtrunk)^-1 * p_world
            pin_foot = np.asarray(oMtrunk.actInv(pin_foot_world), dtype=float).reshape(3)

            err = float(np.linalg.norm(fk_pos - pin_foot))
            per_leg_max[leg_id] = max(per_leg_max[leg_id], err)
            if err > float(worst["err"]):
                worst.update({"leg": leg_id, "q_leg": leg_qs[leg_id], "fk": fk_pos, "pin": pin_foot, "err": err})

    print("=== Foot frame alignment (IK FK vs pinocchio foot_link) ===")
    print(
        "[KNOWN_PRE_STAGE2] lf/lh 足端对齐偏差来自解析 FK 与 Pinocchio/URDF 链的历史建模差（非 Stage2 rail parent 引入）；"
        "Stage2 几何回归以 verify_stage2_rail_geometry.py 为准，本段仅作诊断不导致进程失败。"
    )
    for leg_id in leg_ids:
        e = per_leg_max[leg_id]
        tag = "[KNOWN_PRE_STAGE2] " if leg_id in ("lf", "lh") else ""
        print(f"{tag}{leg_id}: max |p_fk - p_pin| = {e:.6e} m [{_grade(e, th.align_foot_pass_m, th.align_foot_warn_m)}]")
    overall = max(per_leg_max.values())
    print(f"overall: max |p_fk - p_pin| = {overall:.6e} m [{_grade(overall, th.align_foot_pass_m, th.align_foot_warn_m)}]")
    if worst["leg"] is not None:
        print("\nworst-case sample (alignment):")
        print(f"leg={worst['leg']}")
        print(f"q_leg = {worst['q_leg']}")
        print(f"fk_pos  = {np.asarray(worst['fk']).tolist()}")
        print(f"pin_pos = {np.asarray(worst['pin']).tolist()}")
        print(f"err = {float(worst['err']):.6e} m")


def main() -> None:
    random.seed(0)
    np.random.seed(0)
    th = Thresholds()

    print("Running dog2 kinematics regression tests...\n")
    test_fk_ik_roundtrip(num_samples_per_leg=20, th=th)
    print()
    try:
        test_foot_frame_alignment(num_samples=10, th=th)
    except Exception as exc:
        print(f"[ERROR] Foot frame alignment test failed with exception: {exc}")


if __name__ == "__main__":
    main()
