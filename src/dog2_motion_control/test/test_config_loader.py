"""
测试配置文件加载器
"""

import pytest
import tempfile
import os
from pathlib import Path
from dog2_motion_control.config_loader import ConfigLoader
from dog2_motion_control.gait_generator import GaitConfig


class TestConfigLoader:
    """测试配置加载器"""
    
    def test_load_default_config(self):
        """测试加载默认配置"""
        loader = ConfigLoader()
        success = loader.load()
        
        # 即使文件不存在，也应该成功加载（使用默认值）
        assert loader.is_loaded
        
        # 验证可以获取配置
        gait_config = loader.get_gait_config()
        assert isinstance(gait_config, GaitConfig)
        assert gait_config.stride_length > 0
        assert gait_config.stride_height > 0
        assert gait_config.cycle_time > 0
        assert 0 < gait_config.duty_factor < 1.0
    
    def test_load_valid_yaml(self):
        """测试加载有效的YAML文件"""
        # 创建临时配置文件
        valid_config = """
gait:
  stride_length: 0.10
  stride_height: 0.06
  cycle_time: 1.5
  duty_factor: 0.80
  body_height: 0.25
  gait_type: "crawl"

joint_limits:
  rail:
    min: -0.05
    max: 0.05

control:
  frequency: 50.0
  max_joint_velocity: 2.0
"""
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write(valid_config)
            temp_path = f.name
        
        try:
            loader = ConfigLoader(temp_path)
            success = loader.load()
            
            assert success
            assert loader.is_loaded
            
            # 验证加载的值
            gait_config = loader.get_gait_config()
            assert gait_config.stride_length == 0.10
            assert gait_config.stride_height == 0.06
            assert gait_config.cycle_time == 1.5
            assert gait_config.duty_factor == 0.80
            assert gait_config.body_height == 0.25
            assert gait_config.gait_type == "crawl"
            
        finally:
            os.unlink(temp_path)
    
    def test_invalid_stride_length(self):
        """测试无效的步长参数"""
        invalid_config = """
gait:
  stride_length: -0.1
  stride_height: 0.05
  cycle_time: 2.0
  duty_factor: 0.75
  body_height: 0.2
  gait_type: "crawl"

joint_limits:
  rail:
    min: -0.05
    max: 0.05

control:
  frequency: 50.0
  max_joint_velocity: 2.0
"""
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write(invalid_config)
            temp_path = f.name
        
        try:
            loader = ConfigLoader(temp_path)
            success = loader.load()
            
            # 应该加载失败，但使用默认值
            assert not success
            assert loader.is_loaded
            
            # 应该使用默认值
            gait_config = loader.get_gait_config()
            assert gait_config.stride_length == 0.08  # 默认值
            
        finally:
            os.unlink(temp_path)
    
    def test_invalid_duty_factor_for_crawl(self):
        """测试爬行步态的无效duty_factor"""
        invalid_config = """
gait:
  stride_length: 0.08
  stride_height: 0.05
  cycle_time: 2.0
  duty_factor: 0.5
  body_height: 0.2
  gait_type: "crawl"

joint_limits:
  rail:
    min: -0.05
    max: 0.05

control:
  frequency: 50.0
  max_joint_velocity: 2.0
"""
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write(invalid_config)
            temp_path = f.name
        
        try:
            loader = ConfigLoader(temp_path)
            success = loader.load()
            
            # 应该加载失败（爬行步态要求duty_factor >= 0.75）
            assert not success
            assert loader.is_loaded
            
            # 应该使用默认值
            gait_config = loader.get_gait_config()
            assert gait_config.duty_factor == 0.75  # 默认值
            
        finally:
            os.unlink(temp_path)
    
    def test_missing_required_key(self):
        """测试缺少必需键"""
        invalid_config = """
gait:
  stride_length: 0.08
  stride_height: 0.05
  cycle_time: 2.0
  duty_factor: 0.75
  body_height: 0.2
  gait_type: "crawl"

# 缺少 joint_limits 和 control
"""
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write(invalid_config)
            temp_path = f.name
        
        try:
            loader = ConfigLoader(temp_path)
            success = loader.load()
            
            # 应该加载失败
            assert not success
            assert loader.is_loaded
            
            # 应该使用默认配置
            config_data = loader.get_config_data()
            assert 'joint_limits' in config_data
            assert 'control' in config_data
            
        finally:
            os.unlink(temp_path)
    
    def test_get_joint_limits(self):
        """测试获取关节限位"""
        loader = ConfigLoader()
        loader.load()
        
        joint_limits = loader.get_joint_limits()
        
        assert 'rail' in joint_limits
        
        # 验证每个关节都有min和max
        for joint in ['rail']:
            assert 'min' in joint_limits[joint]
            assert 'max' in joint_limits[joint]
            assert joint_limits[joint]['min'] < joint_limits[joint]['max']
    
    def test_get_control_params(self):
        """测试获取控制参数"""
        loader = ConfigLoader()
        loader.load()
        
        control_params = loader.get_control_params()
        
        assert 'frequency' in control_params
        assert 'max_joint_velocity' in control_params
        assert control_params['frequency'] > 0
        assert control_params['max_joint_velocity'] > 0
    
    def test_malformed_yaml(self):
        """测试格式错误的YAML文件"""
        malformed_config = """
gait:
  stride_length: 0.08
  stride_height: 0.05
  cycle_time: 2.0
  duty_factor: 0.75
  body_height: 0.2
  gait_type: "crawl
  # 缺少引号闭合
"""
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write(malformed_config)
            temp_path = f.name
        
        try:
            loader = ConfigLoader(temp_path)
            success = loader.load()
            
            # 应该加载失败，但使用默认值
            assert not success
            assert loader.is_loaded
            
            # 应该可以获取配置
            gait_config = loader.get_gait_config()
            assert isinstance(gait_config, GaitConfig)
            
        finally:
            os.unlink(temp_path)


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
