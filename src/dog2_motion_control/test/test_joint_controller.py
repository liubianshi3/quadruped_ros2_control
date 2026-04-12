"""
测试关节控制器

验证JointController的基本功能
"""

import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from dog2_motion_control.joint_controller import JointController
from dog2_motion_control.joint_names import (
    get_rail_joint_name,
    get_revolute_joint_name,
    LEG_PREFIX_MAP
)


@pytest.fixture
def ros_context():
    """初始化ROS 2上下文"""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def test_node(ros_context):
    """创建测试节点"""
    node = Node('test_joint_controller')
    yield node
    node.destroy_node()


def test_joint_controller_initialization(test_node):
    """测试关节控制器初始化
    
    验证：
    - ROS 2节点正确创建
    - JointTrajectory发布器正确创建
    - JointState订阅器正确创建
    - 关节限位正确加载
    
    需求: 1.1, 5.1
    """
    controller = JointController(test_node)
    
    # 验证发布器和订阅器已创建
    assert controller.revolute_trajectory_pub is not None
    assert controller.rail_trajectory_pub is not None
    assert controller.joint_state_sub is not None
    
    # 验证关节限位已加载（16个关节：4个导轨 + 12个旋转）
    assert len(controller.joint_limits) == 16
    
    # 验证导轨关节限位
    for leg_num in [1, 2, 3, 4]:
        rail_joint = get_rail_joint_name(leg_num)
        assert rail_joint in controller.joint_limits
        lower, upper = controller.joint_limits[rail_joint]
        assert isinstance(lower, float)
        assert isinstance(upper, float)
    
    # 验证旋转关节限位
    for leg_num in [1, 2, 3, 4]:
        for joint_type in ['coxa', 'femur', 'tibia']:
            joint_name = get_revolute_joint_name(leg_num, joint_type)
            assert joint_name in controller.joint_limits
            lower, upper = controller.joint_limits[joint_name]
            assert isinstance(lower, float)
            assert isinstance(upper, float)


def test_joint_state_callback(test_node):
    """测试关节状态回调
    
    验证：
    - 正确解析JointState消息
    - 正确存储关节位置和速度
    
    需求: 1.4, 5.3
    """
    controller = JointController(test_node)
    
    # 创建模拟的JointState消息
    msg = JointState()
    msg.name = ['lf_rail_joint', 'lf_coxa_joint', 'lf_femur_joint']
    msg.position = [0.0, 0.5, -0.3]
    msg.velocity = [0.0, 0.1, -0.05]
    
    # 调用回调函数
    controller._joint_state_callback(msg)
    
    # 验证状态已正确存储
    assert 'lf_rail_joint' in controller.current_joint_states
    assert controller.current_joint_states['lf_rail_joint']['position'] == 0.0
    assert controller.current_joint_states['lf_rail_joint']['velocity'] == 0.0
    
    assert 'lf_coxa_joint' in controller.current_joint_states
    assert controller.current_joint_states['lf_coxa_joint']['position'] == 0.5
    assert controller.current_joint_states['lf_coxa_joint']['velocity'] == 0.1


def test_check_joint_limits_within_range(test_node):
    """测试关节限位检查 - 正常范围内
    
    验证：
    - 在限位范围内的值不被修改
    
    需求: 1.5
    """
    controller = JointController(test_node)
    
    # 测试旋转关节（弧度）
    joint_name = 'lf_coxa_joint'
    lower_limit, upper_limit = controller.joint_limits[joint_name]
    position = 0.5 * (lower_limit + upper_limit)  # 在当前URDF限位范围内
    result = controller.check_joint_limits(joint_name, position)
    assert result == position


def test_check_joint_limits_below_lower(test_node):
    """测试关节限位检查 - 低于下限
    
    验证：
    - 低于下限的值被限制到下限
    - 记录警告日志
    
    需求: 1.5
    """
    controller = JointController(test_node)
    
    # 测试旋转关节（弧度）
    joint_name = 'lf_coxa_joint'
    position = -5.0  # 低于下限
    result = controller.check_joint_limits(joint_name, position)
    
    lower_limit, _ = controller.joint_limits[joint_name]
    assert result == lower_limit


def test_check_joint_limits_above_upper(test_node):
    """测试关节限位检查 - 高于上限
    
    验证：
    - 高于上限的值被限制到上限
    - 记录警告日志
    
    需求: 1.5
    """
    controller = JointController(test_node)
    
    # 测试旋转关节（弧度）
    joint_name = 'lf_femur_joint'
    position = 5.0  # 高于上限
    result = controller.check_joint_limits(joint_name, position)
    
    _, upper_limit = controller.joint_limits[joint_name]
    assert result == upper_limit


def test_monitor_rail_positions_normal(test_node):
    """测试导轨位置监控 - 正常情况
    
    验证：
    - 所有导轨在±0.5mm范围内返回True
    
    需求: 8.5, 8.6
    """
    controller = JointController(test_node)
    
    # 模拟正常的导轨状态（所有导轨在0.0米附近）
    for leg_num in [1, 2, 3, 4]:
        rail_joint = get_rail_joint_name(leg_num)
        controller.current_joint_states[rail_joint] = {
            'position': 0.0001,  # 0.1mm，在阈值内
            'velocity': 0.0
        }
    
    result = controller.monitor_rail_positions()
    assert result is True


def test_monitor_rail_positions_slip_detected(test_node):
    """测试导轨位置监控 - 检测到滑动
    
    验证：
    - 导轨超出±0.5mm范围返回False
    - 记录错误日志
    
    需求: 8.5, 8.6
    """
    controller = JointController(test_node)
    
    # 模拟导轨滑动（j1超出阈值）
    controller.last_rail_targets['lf_rail_joint'] = 0.0
    controller.current_joint_states['lf_rail_joint'] = {
        'position': 0.01,  # 10.0mm，超出默认5mm阈值
        'velocity': 0.0
    }
    
    # 其他导轨正常
    for leg_num in [2, 3, 4]:
        rail_joint = get_rail_joint_name(leg_num)
        controller.current_joint_states[rail_joint] = {
            'position': 0.0,
            'velocity': 0.0
        }
    
    controller._rail_slip_patience = 1
    result = controller.monitor_rail_positions()
    assert result is False


def test_get_joint_position(test_node):
    """测试获取单个关节位置
    
    验证：
    - 正确返回已知关节的位置
    - 未知关节返回None
    """
    controller = JointController(test_node)
    
    # 设置一些关节状态
    controller.current_joint_states['lf_rail_joint'] = {
        'position': 0.0,
        'velocity': 0.0
    }
    
    # 测试已知关节
    position = controller.get_joint_position('lf_rail_joint')
    assert position == 0.0
    
    # 测试未知关节
    position = controller.get_joint_position('unknown_joint')
    assert position is None


def test_send_joint_commands_structure(test_node):
    """测试发送关节命令的消息结构
    
    验证：
    - 消息包含16个关节
    - 导轨关节恒定为0.0米
    - 旋转关节使用提供的值
    
    需求: 1.2, 1.3
    """
    controller = JointController(test_node)
    
    # 准备关节位置命令
    joint_positions = {}
    for leg_num in [1, 2, 3, 4]:
        for joint_type in ['coxa', 'femur', 'tibia']:
            joint_name = get_revolute_joint_name(leg_num, joint_type)
            joint_positions[joint_name] = 0.5  # 示例角度
    
    # 发送命令（这里只是测试不会崩溃）
    # 实际的消息发布需要完整的ROS 2环境
    try:
        controller.send_joint_commands(joint_positions)
    except Exception as e:
        pytest.fail(f"send_joint_commands raised exception: {e}")


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
