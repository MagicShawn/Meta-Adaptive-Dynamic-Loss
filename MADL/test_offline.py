import time
import numpy as np
import torch
import os
import yaml

# Adjust path to import correctly
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), 'policy'))
from backbone import InferenceBackbone

def main():
    print("=== 启动第一阶段：纯离线测试 (Mock / 伪数据测试) ===")
    
    # Configuration paths
    base_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(base_dir, 'config/config.yaml')
    
    # Load config to get realistic parameters
    try:
        with open(config_path, 'r') as f:
            cfg = yaml.safe_load(f)
        net_weight = os.path.join(base_dir, cfg.get('net_weight', 'policy/models/worker_ckpt_017999.pth'))
        norm_ckpt = os.path.join(base_dir, cfg.get('norm_ckpt', 'policy/models/norm_ckpt_017999.pth'))
        no_odom = cfg.get('no_odom', True)
        depth_scale = cfg.get('depth_scale', 1000.0)
        margin = cfg.get('margin', 0.2)
        target_speed = cfg.get('target_speed', 1.0)
        rate_hz = cfg.get('rate_hz', 50)
    except Exception as e:
        print(f"Warning: Could not load config.yaml perfectly, using defaults. {e}")
        net_weight = os.path.join(base_dir, 'policy/models/worker_ckpt_017999.pth')
        norm_ckpt = os.path.join(base_dir, 'policy/models/norm_ckpt_017999.pth')
        no_odom = True
        depth_scale = 1000.0
        margin = 0.2
        target_speed = 1.0
        rate_hz = 50

    print(f"Loading weights:\n  - Net: {net_weight}\n  - Norm: {norm_ckpt}")
    
    # Instantiate the backbone
    try:
        backbone = InferenceBackbone(
            net_weight=net_weight,
            norm_ckpt=norm_ckpt,
            no_odom=no_odom,
            depth_scale=depth_scale
        )
        print("[V] 模型权重与归一化参数加载成功！")
    except Exception as e:
        print(f"[X] 模型加载失败: {e}")
        return

    # Create dummy datam
    # Simulating a depth image (e.g., RealSense 480x640, values in mm)
    # The depth image cb originally gives a numpy array.
    dummy_depth = np.random.randint(0, 5000, size=(480, 640), dtype=np.uint16)
    
    dummy_odom = {
        'position': [0.0, 0.0, 1.0],
        'quaternion': [1.0, 0.0, 0.0, 0.0], # w, x, y, z
        'linear_velocity': [0.5, 0.0, 0.0]
    }
    
    dummy_target = {
        'position': [5.0, 0.0, 1.0]
    }

    # Test single forward pass
    print("\n--- 执行单次前向传播测试 ---")
    try:
        # Predict API: predict(self, depth_img, odom, target, margin, target_speed)
        des_acc, des_vel, _ = backbone.predict(dummy_depth, dummy_odom, dummy_target, margin, target_speed)
        print(f"[V] 前向传播成功!")
        print(f"  - 输出加速度 (des_acc): {des_acc}, 形状: {des_acc.shape}")
        print(f"  - 输出速度 (des_vel): {des_vel}, 形状: {des_vel.shape}")
        
        if np.isnan(des_acc).any() or np.isnan(des_vel).any():
            print("[X] 警告: 输出包含 NaN!")
        else:
            print("[V] 输出无异常值 NaN。")
            
    except Exception as e:
        print(f"[X] 前向传播失败: {e}")
        return

    # Profiling 1000 iterations
    iterations = 1000
    print(f"\n--- 执行性能基准测试 (Profiling, {iterations} 次循环) ---")
    
    # Warmup
    for _ in range(10):
        backbone.predict(dummy_depth, dummy_odom, dummy_target, margin, target_speed)
        
    start_time = time.time()
    for _ in range(iterations):
        backbone.predict(dummy_depth, dummy_odom, dummy_target, margin, target_speed)
    end_time = time.time()
    
    total_time = end_time - start_time
    avg_time_ms = (total_time / iterations) * 1000
    fps = iterations / total_time
    required_ms = 1000.0 / rate_hz
    
    print(f"总耗时: {total_time:.3f} 秒")
    print(f"平均每次推理耗时: {avg_time_ms:.2f} ms")
    print(f"估计最高运行频率: {fps:.2f} Hz")
    
    print(f"\n系统要求:")
    print(f"  - 配置文件要求频率: {rate_hz} Hz ({required_ms:.2f} ms 预算)")
    if avg_time_ms < required_ms:
        print(f"[V] 性能及格！当前平均推理耗时 {avg_time_ms:.2f}ms 满足 {required_ms:.2f}ms 的要求。")
    else:
        print(f"[X] 性能警告！当前耗时 {avg_time_ms:.2f}ms 超过了要求的 {required_ms:.2f}ms，可能会导致控制卡顿。")

if __name__ == '__main__':
    main()
