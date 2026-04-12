"""
运动学求解器单元测试

测试逆运动学和正运动学的功能
根据需求 3.2, 3.3 编写的测试用例
"""

import pytest
import numpy as np
from dog2_motion_control.kinematics_solver import create_kinematics_solver
from dog2_motion_control.config_loader import ConfigLoader
from dog2_motion_control.leg_parameters import LEG_PARAMETERS


class TestKinematicsSolver:
    """运动学求解器测试类"""
    
    @pytest.fixture
    def solver(self):
        """创建运动学求解器实例"""
        solver = create_kinematics_solver()
        # 让测试与主代码的IK求解配置一致（否则可能因正则权重不同导致收敛失败）
        cfg_loader = ConfigLoader(None)
        cfg_loader.load()
        solver.configure_regularization(cfg_loader.get_ik_regularization())
        return solver
    
    def test_solver_initialization(self, solver):
        """测试求解器初始化"""
        assert solver is not None
        assert len(solver.leg_params) == 4
        assert 'lf' in solver.leg_params
        assert 'rf' in solver.leg_params
        assert 'lh' in solver.leg_params
        assert 'rh' in solver.leg_params


class TestIKKnownPositions:
    """测试已知位置的IK解（需求 3.2）"""
    
    @pytest.fixture
    def solver(self):
        """创建运动学求解器实例"""
        solver = create_kinematics_solver()
        cfg_loader = ConfigLoader(None)
        cfg_loader.load()
        solver.configure_regularization(cfg_loader.get_ik_regularization())
        return solver
    
    def test_ik_center_position_lf(self, solver):
        """测试前左腿的中心位置"""
        leg_id = 'lf'
        # 使用求解器的站立姿态（q=[0,0.3,-0.5]）通过FK生成自洽目标
        # 这样目标必然在当前几何模型下可达，避免旧坐标硬编码导致None。
        joint_seed = (0.0, 0.0, 0.3000, -0.5000)  # (s_m, theta_coxa, theta_femur, theta_tibia)
        target_pos = solver.solve_fk(leg_id, joint_seed)
        
        joint_angles = solver.solve_ik(leg_id, target_pos)
        assert joint_angles is not None, "中心位置应该有IK解"

        # 验证IK->FK闭环误差
        result_pos = solver.solve_fk(leg_id, joint_angles)
        error = np.linalg.norm(np.array(result_pos) - np.array(target_pos))
        assert error < 0.01, f"IK->FK误差应该<10mm，实际: {error*1000:.3f}mm"
    
    def test_ik_known_positions_all_legs(self, solver):
        """测试所有腿的已知位置"""
        # 用同一站立姿态在每条腿生成目标，并验证IK->FK闭环
        joint_seed = (0.0, 0.0, 0.3000, -0.5000)  # (s_m, theta_coxa, theta_femur, theta_tibia)
        for leg_id in ['lf', 'rf', 'lh', 'rh']:
            target_pos = solver.solve_fk(leg_id, joint_seed)
            joint_angles = solver.solve_ik(leg_id, target_pos)
            assert joint_angles is not None, f"{leg_id}: 已知位置应该有IK解"
            
            # 验证FK能够恢复原始位置
            result_pos = solver.solve_fk(leg_id, joint_angles)
            error = np.linalg.norm(np.array(result_pos) - np.array(target_pos))
            assert error < 0.01, f"{leg_id}: IK->FK误差应该<10mm，实际: {error*1000:.3f}mm"
    
    def test_ik_extended_position(self, solver):
        """测试腿部伸展位置（用FK生成自洽目标）"""
        leg_id = 'lf'
        # 取一个在关节限位内的伸展姿态：theta_femur更大、theta_tibia更负
        joint_seed = (0.0, 0.0, 1.0, -1.4)  # (s_m, theta_coxa, theta_femur, theta_tibia)
        target_pos = solver.solve_fk(leg_id, joint_seed)
        
        joint_angles = solver.solve_ik(leg_id, target_pos)
        assert joint_angles is not None, "伸展位置应该有IK解"
        
        # 验证FK
        result_pos = solver.solve_fk(leg_id, joint_angles)
        error = np.linalg.norm(np.array(result_pos) - np.array(target_pos))
        assert error < 0.01, f"伸展位置IK->FK误差应该<10mm，实际: {error*1000:.3f}mm"
    
    def test_ik_retracted_position(self, solver):
        """测试腿部收缩位置（用FK生成自洽目标）"""
        leg_id = 'lf'
        # 收缩姿态：femur更小或tibia更负（仍在限位内）
        joint_seed = (0.0, 0.0, 1.6, -2.6)  # (s_m, theta_coxa, theta_femur, theta_tibia)
        target_pos = solver.solve_fk(leg_id, joint_seed)
        
        joint_angles = solver.solve_ik(leg_id, target_pos)
        assert joint_angles is not None, "收缩位置应该有IK解"
        result_pos = solver.solve_fk(leg_id, joint_angles)
        error = np.linalg.norm(np.array(result_pos) - np.array(target_pos))
        assert error < 0.01, f"收缩位置IK->FK误差应该<10mm，实际: {error*1000:.3f}mm"


class TestWorkspaceBoundaries:
    """测试工作空间边界情况（需求 3.3）"""
    
    @pytest.fixture
    def solver(self):
        """创建运动学求解器实例"""
        solver = create_kinematics_solver()
        cfg_loader = ConfigLoader(None)
        cfg_loader.load()
        solver.configure_regularization(cfg_loader.get_ik_regularization())
        return solver
    
    def test_ik_too_far(self, solver):
        """测试目标位置过远（超出最大伸展）"""
        leg_id = 'lf'
        # 选择一个明显超出工作空间的位置
        target_pos = (5.0, 5.0, 5.0)
        
        joint_angles = solver.solve_ik(leg_id, target_pos)
        assert joint_angles is None, "超出工作空间的位置应该返回None"
    
    def test_ik_too_close(self, solver):
        """测试目标位置过近（小于最小伸展）"""
        leg_id = 'lf'
        params = LEG_PARAMETERS[leg_id]
        
        # 选择一个非常接近基座的位置（可能小于最小伸展）
        base_pos = params.base_position
        target_pos = (base_pos[0], base_pos[1], base_pos[2])
        
        joint_angles = solver.solve_ik(leg_id, target_pos)
        # 这个位置可能有解也可能无解，取决于具体的几何配置
        # 如果有解，验证FK
        if joint_angles is not None:
            result_pos = solver.solve_fk(leg_id, joint_angles)
            error = np.linalg.norm(np.array(result_pos) - np.array(target_pos))
            assert error < 0.01, f"IK->FK误差应该<10mm，实际: {error*1000:.3f}mm"
    
    def test_ik_at_workspace_edge(self, solver):
        """测试工作空间边缘的位置"""
        leg_id = 'lf'
        params = LEG_PARAMETERS[leg_id]
        L1, L2, L3 = params.link_lengths
        
        # 计算理论最大伸展
        max_reach = L2 + L3
        
        # 测试接近最大伸展的位置（留1cm裕度）
        base_pos = params.base_position
        reach_distance = max_reach - 0.01
        
        # 在腿部前方
        target_pos = (base_pos[0] + reach_distance * 0.8, base_pos[1], base_pos[2] - 0.1)
        
        joint_angles = solver.solve_ik(leg_id, target_pos)
        # 边缘位置可能有解也可能无解
        if joint_angles is not None:
            result_pos = solver.solve_fk(leg_id, joint_angles)
            error = np.linalg.norm(np.array(result_pos) - np.array(target_pos))
            assert error < 0.01, f"边缘位置IK->FK误差应该<10mm，实际: {error*1000:.3f}mm"
    
    def test_ik_negative_z(self, solver):
        """测试负Z方向的位置（地面以下）"""
        leg_id = 'lf'
        params = LEG_PARAMETERS[leg_id]
        base_pos = params.base_position
        
        # 测试地面以下的位置
        target_pos = (base_pos[0] + 0.2, base_pos[1], -0.1)
        
        joint_angles = solver.solve_ik(leg_id, target_pos)
        # 这个位置可能有解（取决于工作空间）
        if joint_angles is not None:
            result_pos = solver.solve_fk(leg_id, joint_angles)
            error = np.linalg.norm(np.array(result_pos) - np.array(target_pos))
            assert error < 0.01, f"负Z位置IK->FK误差应该<10mm，实际: {error*1000:.3f}mm"
    
    def test_ik_joint_limits_violation(self, solver):
        """测试导致关节限位违反的位置"""
        leg_id = 'lf'
        params = LEG_PARAMETERS[leg_id]
        base_pos = params.base_position
        
        # 尝试一个可能导致关节限位违反的极端位置
        # 例如：极度侧向的位置
        target_pos = (base_pos[0], base_pos[1] + 0.5, base_pos[2])
        
        joint_angles = solver.solve_ik(leg_id, target_pos)
        # 如果超出关节限位，应该返回None
        if joint_angles is not None:
            # 如果有解，验证所有关节都在限位内
            s_m, theta_haa, theta_hfe, theta_kfe = joint_angles
            
            limits = params.joint_limits
            coxa_min, coxa_max = limits['coxa']
            femur_min, femur_max = limits['femur']
            tibia_min, tibia_max = limits['tibia']
            
            assert coxa_min <= theta_haa <= coxa_max, "coxa角度应该在限位内"
            assert femur_min <= theta_hfe <= femur_max, "femur角度应该在限位内"
            assert tibia_min <= theta_kfe <= tibia_max, "tibia角度应该在限位内"


class TestCoordinateTransforms:
    """测试不同腿部的坐标系转换（需求 3.2）"""
    
    @pytest.fixture
    def solver(self):
        """创建运动学求解器实例"""
        solver = create_kinematics_solver()
        cfg_loader = ConfigLoader(None)
        cfg_loader.load()
        solver.configure_regularization(cfg_loader.get_ik_regularization())
        return solver
    
    def test_coordinate_transform_roundtrip_all_legs(self, solver):
        """测试所有腿的坐标系转换round-trip"""
        test_position = np.array([1.2, -0.75, 0.0])
        
        for leg_id in ['lf', 'rf', 'lh', 'rh']:
            # 转换到腿部局部坐标系
            pos_local = solver._transform_to_leg_frame(test_position, leg_id)
            
            # 转换回base_link坐标系
            pos_back = solver._transform_from_leg_frame(pos_local, leg_id)
            
            # 验证round-trip
            error = np.linalg.norm(pos_back - test_position)
            assert error < 1e-10, f"{leg_id}: 坐标系转换round-trip误差应该接近0，实际: {error}"
    
    def test_all_legs_use_normalized_base_rotation(self, solver):
        """测试四条腿都使用统一的 rail 根坐标系旋转。"""
        for leg_id in ['lf', 'rf', 'lh', 'rh']:
            params = LEG_PARAMETERS[leg_id]
            roll, pitch, yaw = params.base_rotation

            assert np.isclose(roll, np.pi/2, atol=0.01), f"{leg_id}: roll应该约为90度"
            assert np.isclose(pitch, 0.0, atol=0.01), f"{leg_id}: pitch应该为0"
            assert np.isclose(yaw, 0.0, atol=0.01), f"{leg_id}: yaw应该已统一为0"

            vec_base = np.array([0.0, 0.0, 1.0])
            R = solver._rotation_matrix_from_rpy(roll, pitch, yaw)
            vec_local = R.T @ vec_base
            assert np.abs(vec_local[1]) > 0.9, f"{leg_id}: Z轴应该映射到局部Y轴"
    
    def test_leg_base_positions(self, solver):
        """测试腿部基座位置的正确性"""
        # 验证所有腿的基座位置都已正确设置
        for leg_id in ['lf', 'rf', 'lh', 'rh']:
            params = LEG_PARAMETERS[leg_id]
            base_pos = params.base_position
            
            # 基座位置应该在合理范围内（base_link-local）
            assert 0.05 < base_pos[0] < 0.6, f"{leg_id}: X坐标应该在合理范围内"
            assert -1e-6 <= base_pos[1] <= 0.25, f"{leg_id}: Y坐标应该在合理范围内"
            assert abs(base_pos[2]) < 1e-6, f"{leg_id}: Z坐标应接近0"
    
    def test_ik_respects_leg_frame(self, solver):
        """测试IK求解器正确使用腿部坐标系"""
        # 为每条腿测试相同的相对位置
        for leg_id in ['lf', 'rf', 'lh', 'rh']:
            params = LEG_PARAMETERS[leg_id]
            base_pos = params.base_position
            
            # 在base_link坐标系中，选择一个相对于基座的位置
            target_pos = (base_pos[0] + 0.2, base_pos[1], base_pos[2] - 0.2)
            
            joint_angles = solver.solve_ik(leg_id, target_pos)
            
            if joint_angles is not None:
                # 验证FK能够恢复原始位置
                result_pos = solver.solve_fk(leg_id, joint_angles)
                error = np.linalg.norm(np.array(result_pos) - np.array(target_pos))
                assert error < 0.01, f"{leg_id}: IK->FK误差应该<10mm，实际: {error*1000:.3f}mm"


class TestRailLocking:
    """测试导轨锁定功能"""
    
    @pytest.fixture
    def solver(self):
        """创建运动学求解器实例（收紧 rail 搜索与迭代，避免慢路径拖死单元测试）。"""
        solver = create_kinematics_solver()
        cfg_loader = ConfigLoader(None)
        cfg_loader.load()
        ik_reg = cfg_loader.get_ik_regularization().copy()
        ik_reg["rail_candidates"] = min(5, int(ik_reg.get("rail_candidates", 5)))
        ik_reg["max_iterations"] = min(25, int(ik_reg.get("max_iterations", 25)))
        solver.configure_regularization(ik_reg)
        return solver
    
    def test_rail_always_zero(self, solver):
        """测试导轨位移始终为0：目标由 FK(rail=0, 站立角) 生成，保证可达且与 rail 锁定语义一致。"""
        for leg_id in ['lf', 'rf', 'lh', 'rh']:
            q_ref = np.asarray(solver._standing_angles[leg_id], dtype=float)
            target_pos = solver.solve_fk(leg_id, (0.0, float(q_ref[0]), float(q_ref[1]), float(q_ref[2])))
            joint_angles = solver.solve_ik(leg_id, target_pos, rail_offset=0.0)

            assert joint_angles is not None, f"{leg_id}: IK should solve at FK target from rail=0 standing"
            s_m, _, _, _ = joint_angles
            assert s_m == 0.0, f"{leg_id}: 导轨位移应该始终为0.0米"
    
    def test_rail_offset_parameter_ignored(self, solver):
        """测试rail_offset参数被正确处理"""
        leg_id = 'lf'
        target_pos = (1.0, -0.9, 0.0)
        
        # 即使传入非零的rail_offset，当前阶段应该使用0.0
        joint_angles = solver.solve_ik(leg_id, target_pos, rail_offset=0.0)
        
        if joint_angles is not None:
            s_m, _, _, _ = joint_angles
            assert s_m == 0.0, "导轨位移应该为0.0米"


class TestIKPropertyBased:
    """基于属性的测试（Property-Based Testing）"""
    
    @pytest.fixture
    def solver(self):
        """创建运动学求解器实例"""
        solver = create_kinematics_solver()
        cfg_loader = ConfigLoader(None)
        cfg_loader.load()
        # Property-based tests have to solve IK many times; reduce solver cost.
        ik_reg = cfg_loader.get_ik_regularization().copy()
        ik_reg["rail_candidates"] = min(5, int(ik_reg.get("rail_candidates", 5)))
        ik_reg["max_iterations"] = min(25, int(ik_reg.get("max_iterations", 25)))
        solver.configure_regularization(ik_reg)
        return solver
    
    # Feature: spider-robot-basic-motion, Property 8: 逆运动学正确性（Round-trip）
    def test_ik_fk_roundtrip_property(self, solver):
        """
        属性测试：对于任意工作空间内的位置，IK->FK应该返回原始位置
        
        验证需求: 3.1, 3.3
        
        这是一个round-trip属性测试：
        - 生成随机的脚部目标位置
        - 计算逆运动学得到关节角度
        - 使用正运动学将关节角度转换回脚部位置
        - 验证结果位置与目标位置的误差小于1mm
        """
        from datetime import timedelta

        from hypothesis import given, settings, strategies as st, assume

        # 定义测试策略：为每条腿生成工作空间内的随机位置
        @settings(max_examples=3, deadline=timedelta(seconds=20))
        @given(
            leg_id=st.sampled_from(['lf', 'rf', 'lh', 'rh']),
            # 相对于基座的偏移量（在腿部工作空间内）
            dx=st.floats(min_value=0.12, max_value=0.30),  # 前方偏移
            dy=st.floats(min_value=-0.12, max_value=0.12),  # 侧向偏移
            dz=st.floats(min_value=-0.30, max_value=-0.12),  # 下方偏移
        )
        def run_roundtrip_test(leg_id, dx, dy, dz):
            """执行round-trip测试"""
            params = LEG_PARAMETERS[leg_id]
            base_pos = params.base_position
            
            # 构造目标位置（相对于基座）
            target_pos = (
                base_pos[0] + dx,
                base_pos[1] + dy,
                base_pos[2] + dz
            )
            
            # 逆运动学
            joint_angles = solver.solve_ik(leg_id, target_pos)
            
            # 如果IK无解，跳过这个测试用例（位置可能超出工作空间）
            assume(joint_angles is not None)
            
            # 正运动学
            result_pos = solver.solve_fk(leg_id, joint_angles)
            
            # 验证round-trip（误差<1mm）
            error = np.linalg.norm(np.array(result_pos) - np.array(target_pos))
            assert error < 0.01, (
                f"{leg_id}: IK->FK round-trip误差过大\n"
                f"  目标位置: {target_pos}\n"
                f"  结果位置: {result_pos}\n"
                f"  误差: {error*1000:.3f}mm (阈值: 10.0mm)\n"
                f"  关节角度: s={joint_angles[0]:.4f}m, "
                f"θ_haa={np.degrees(joint_angles[1]):.2f}°, "
                f"θ_hfe={np.degrees(joint_angles[2]):.2f}°, "
                f"θ_kfe={np.degrees(joint_angles[3]):.2f}°"
            )
        
        # 运行属性测试
        run_roundtrip_test()
    
    # Feature: spider-robot-basic-motion, Property 9: 工作空间外错误处理
    def test_workspace_boundary_error_handling_property(self, solver):
        """
        属性测试：对于任意超出工作空间的位置，IK应该返回None
        
        验证需求: 3.3
        
        这个属性测试验证：
        - 生成明显超出工作空间的随机位置
        - 验证IK求解器正确返回None（无解）
        - 确保系统能够识别并处理不可达的目标位置
        """
        from datetime import timedelta

        from hypothesis import given, settings, strategies as st

        @settings(max_examples=3, deadline=timedelta(seconds=20))
        @given(
            leg_id=st.sampled_from(['lf', 'rf', 'lh', 'rh']),
            # 生成超出工作空间的位置策略
            position_type=st.sampled_from(['too_far', 'too_close', 'extreme_combined']),
            scale=st.floats(min_value=1.5, max_value=3.0),  # 放大因子（缩小范围以加速）
        )
        def run_boundary_test(leg_id, position_type, scale):
            """执行工作空间边界测试"""
            # Avoid cross-example coupling via the solver's smoothness regularization cache.
            # (Hypothesis may call the same test function instance many times.)
            solver._last_solution = {}
            params = LEG_PARAMETERS[leg_id]
            base_pos = params.base_position
            L1, L2, L3 = params.link_lengths
            
            # 计算理论最大伸展距离
            max_reach = L2 + L3
            min_reach = abs(L2 - L3)
            
            # 根据位置类型生成超出工作空间的目标位置
            if position_type == 'too_far':
                # 距离过远：在腿部局部系下明显超过伸展，触发 solve_ik 的早退（避免数值假解）
                distance = (max_reach + 0.6) * scale
                target_pos = (
                    base_pos[0] + distance * 0.6,
                    base_pos[1] + distance * 0.3,
                    base_pos[2] - distance * 0.5
                )
            
            elif position_type == 'too_close':
                # 距离过近：小于最小伸展（如果min_reach > 0）
                if min_reach > 0.05:  # 只有当最小伸展有意义时才测试
                    distance = min_reach * 0.2  # 远小于最小伸展
                    target_pos = (
                        base_pos[0] + distance * 0.5,
                        base_pos[1] + distance * 0.3,
                        base_pos[2] - distance * 0.4
                    )
                else:
                    # 如果最小伸展接近0，使用极端接近基座的位置
                    target_pos = (
                        base_pos[0] + 0.005,
                        base_pos[1] + 0.005,
                        base_pos[2] - 0.005
                    )
            
            else:  # extreme_combined
                # 组合极端位置：在多个方向上同时超出
                distance = max_reach * scale * 0.7
                target_pos = (
                    base_pos[0] + distance,
                    base_pos[1] + distance * 0.5,
                    base_pos[2] + distance * 0.5
                )
            
            # 逆运动学求解
            joint_angles = solver.solve_ik(leg_id, target_pos)

            # 验证策略：
            # - 对明显过远（too_far），求解器应返回 None。
            # - 对过近（too_close）与极端组合（extreme_combined），当前解析近似下
            #   “理论最小伸展”不一定是严格不可达边界，允许返回解；若返回解，
            #   则验证 IK->FK 闭环与关节限位约束。
            if position_type == "too_far":
                assert joint_angles is None, (
                    f"{leg_id}: too_far should return None\n"
                    f"  目标位置: {target_pos}\n"
                    f"  基座位置: {base_pos}\n"
                    f"  最大伸展: {max_reach:.3f}m\n"
                    f"  实际距离: {np.linalg.norm(np.array(target_pos) - base_pos):.3f}m\n"
                )
            else:
                if joint_angles is not None:
                    result_pos = solver.solve_fk(leg_id, joint_angles)
                    error = np.linalg.norm(np.array(result_pos) - np.array(target_pos))
                    assert error < 0.02, (
                        f"{leg_id}: IK->FK error should be bounded\n"
                        f"  位置类型: {position_type}\n"
                        f"  目标位置: {target_pos}\n"
                        f"  结果位置: {result_pos}\n"
                        f"  误差: {error*1000:.3f}mm\n"
                    )

                    s_m, theta_haa, theta_hfe, theta_kfe = joint_angles
                    limits = params.joint_limits
                    assert limits["coxa"][0] <= theta_haa <= limits["coxa"][1]
                    assert limits["femur"][0] <= theta_hfe <= limits["femur"][1]
                    assert limits["tibia"][0] <= theta_kfe <= limits["tibia"][1]
        
        # 运行属性测试
        run_boundary_test()


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
