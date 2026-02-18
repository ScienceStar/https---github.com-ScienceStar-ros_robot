#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
轨迹数据分析和可视化工具
从JSON文件读取轨迹数据并进行分析
"""

import json
import os
from pathlib import Path
import math


def analyze_trajectory_file(filepath):
    """分析单个轨迹文件"""
    
    with open(filepath, 'r') as f:
        data = json.load(f)
    
    metadata = data['metadata']
    trajectory = data['data']
    
    print("\n" + "=" * 70)
    print(f"【文件: {Path(filepath).name}】")
    print("=" * 70)
    
    print(f"\n📝 元数据:")
    print(f"  轨迹类型:      {metadata['type']}")
    print(f"  最大速度:      {metadata['max_velocity']} m/s")
    print(f"  半径:          {metadata['radius']} m")
    print(f"  采样频率:      {metadata['sim_freq']} Hz")
    print(f"  时间戳:        {metadata['timestamp']}")
    print(f"  总时长:        {metadata['duration']:.2f} 秒")
    print(f"  数据点数:      {metadata['points_count']}")
    
    # 提取数据
    times = [p['time'] for p in trajectory]
    xs = [p['x'] for p in trajectory]
    ys = [p['y'] for p in trajectory]
    speeds = [p.get('speed', 0) for p in trajectory]
    omegas = [p.get('omega', 0) for p in trajectory]
    
    # 位置分析
    print(f"\n📍 位置分析:")
    print(f"  X范围:         [{min(xs):.4f}, {max(xs):.4f}] 米")
    print(f"  Y范围:         [{min(ys):.4f}, {max(ys):.4f}] 米")
    print(f"  起始位置:      ({xs[0]:.4f}, {ys[0]:.4f})")
    print(f"  结束位置:      ({xs[-1]:.4f}, {ys[-1]:.4f})")
    
    # 位移和路程
    displacement = math.sqrt((xs[-1]-xs[0])**2 + (ys[-1]-ys[0])**2)
    total_distance = 0.0
    for i in range(1, len(xs)):
        dx = xs[i] - xs[i-1]
        dy = ys[i] - ys[i-1]
        total_distance += math.sqrt(dx**2 + dy**2)
    
    print(f"\n🚗 运动分析:")
    print(f"  位移:          {displacement:.4f} 米")
    print(f"  总路程:        {total_distance:.4f} 米")
    if total_distance > 0:
        efficiency = displacement / total_distance
        print(f"  效率比:        {efficiency:.4f} ({efficiency*100:.2f}%)")
    
    # 速度分析
    print(f"\n⚡ 速度分析:")
    print(f"  平均速度:      {sum(speeds)/len(speeds):.4f} m/s")
    print(f"  最大速度:      {max(speeds):.4f} m/s")
    print(f"  最小速度:      {min(speeds):.4f} m/s")
    
    # 角速度分析
    print(f"\n🔄 角速度分析:")
    print(f"  平均角速度:    {sum(omegas)/len(omegas):.4f} rad/s")
    print(f"  最大角速度:    {max(omegas):.4f} rad/s")
    print(f"  最小角速度:    {min(omegas):.4f} rad/s")
    
    # 曲率分析（简单估计）
    if len(speeds) > 2:
        curvatures = []
        for i in range(1, len(omegas)-1):
            if speeds[i] > 0.001:  # 避免除以0
                curvature = omegas[i] / speeds[i]
                curvatures.append(abs(curvature))
        
        if curvatures:
            print(f"\n🔸 曲率分析:")
            print(f"  平均曲率:      {sum(curvatures)/len(curvatures):.4f}")
            print(f"  最大曲率:      {max(curvatures):.4f}")


def analyze_all_trajectories(directory='/root/ros_ws/trajectories'):
    """分析所有轨迹文件"""
    
    print("\n" + "╔" + "="*68 + "╗")
    print("║" + "【轨迹数据分析工具】".center(68) + "║")
    print("╚" + "="*68 + "╝")
    
    # 查找所有JSON文件
    json_files = sorted(Path(directory).glob('*.json'))
    
    if not json_files:
        print(f"\n⚠️  未找到轨迹文件: {directory}")
        return
    
    print(f"\n找到 {len(json_files)} 个轨迹文件:")
    for i, f in enumerate(json_files, 1):
        print(f"  {i}. {f.name}")
    
    # 分析每一个
    for filepath in json_files:
        try:
            analyze_trajectory_file(filepath)
        except Exception as e:
            print(f"\n❌ 错误: 无法分析 {filepath.name}: {e}")
    
    # 汇总比较
    print("\n\n" + "="*70)
    print("【轨迹对比总结】".center(70))
    print("="*70)
    
    summary = []
    for filepath in json_files:
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        trajectory = data['data']
        metadata = data['metadata']
        
        xs = [p['x'] for p in trajectory]
        ys = [p['y'] for p in trajectory]
        
        displacement = math.sqrt((xs[-1]-xs[0])**2 + (ys[-1]-ys[0])**2)
        total_distance = 0.0
        for i in range(1, len(xs)):
            total_distance += math.sqrt((xs[i]-xs[i-1])**2 + (ys[i]-ys[i-1])**2)
        
        speeds = [p.get('speed', 0) for p in trajectory]
        
        summary.append({
            'type': metadata['type'],
            'file': Path(filepath).name,
            'points': len(trajectory),
            'duration': metadata['duration'],
            'displacement': displacement,
            'distance': total_distance,
            'avg_speed': sum(speeds)/len(speeds) if speeds else 0,
            'max_speed': max(speeds) if speeds else 0,
        })
    
    # 打印表格
    print(f"\n{'类型':<10} {'数据点':<8} {'时长':<8} {'位移':<8} {'路程':<8} {'平均速':<8} {'最大速':<8}")
    print("-" * 70)
    for s in summary:
        print(f"{s['type']:<10} {s['points']:<8} {s['duration']:<8.2f} "
              f"{s['displacement']:<8.4f} {s['distance']:<8.4f} "
              f"{s['avg_speed']:<8.4f} {s['max_speed']:<8.4f}")
    
    print("\n" + "="*70)
    print("✓ 分析完成！\n")


if __name__ == '__main__':
    analyze_all_trajectories()
