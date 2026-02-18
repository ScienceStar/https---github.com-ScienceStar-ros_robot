#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
多轨迹演示脚本
"""

import sys
sys.path.insert(0, '/root/ros_ws')

from run_standalone import TrajectorySimulatorStandalone

def run_all_trajectories():
    """运行所有轨迹类型的演示"""
    
    trajectories = [
        {'type': 'circle', 'duration': 30.0, 'desc': '圆形轨迹'},
        {'type': 'figure8', 'duration': 30.0, 'desc': '8字形轨迹'},
        {'type': 'sine', 'duration': 20.0, 'desc': '正弦波轨迹'},
        {'type': 'line', 'duration': 30.0, 'desc': '直线轨迹'},
    ]
    
    for i, traj_config in enumerate(trajectories, 1):
        print(f"\n\n{'='*70}")
        print(f"【演示 {i}/{len(trajectories)}: {traj_config['desc']}】".center(70))
        print(f"{'='*70}")
        
        # 创建模拟器
        simulator = TrajectorySimulatorStandalone(
            trajectory_type=traj_config['type'],
            max_velocity=0.5 if traj_config['type'] != 'line' else 1.0,
            radius=2.0,
            sim_freq=10.0
        )
        
        # 运行模拟
        simulator.simulate_trajectory(duration=traj_config['duration'])
        
        # 统计信息
        simulator.print_statistics()
        
        # 可视化
        simulator.visualize_ascii()
        
        # 保存
        simulator.save_trajectory()


if __name__ == '__main__':
    print("\n" + "="*70)
    print("【ROS机器人轨迹模拟 - 多轨迹演示】".center(70))
    print("="*70)
    print("\n将依次演示所有4种轨迹类型...\n")
    
    try:
        run_all_trajectories()
        print("\n" + "="*70)
        print("【所有演示完成！】".center(70))
        print("="*70)
        print("\n轨迹数据已保存到: /root/ros_ws/trajectories/")
        print("\n")
    except KeyboardInterrupt:
        print("\n\n⚠️  演示被中断")
