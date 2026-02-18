#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
独立轨迹模拟器 - 不依赖ROS，纯Python实现
可视化机器人的运动轨迹
"""

import math
import time
import json
from datetime import datetime
from pathlib import Path


class TrajectorySimulatorStandalone:
    """独立轨迹模拟器"""
    
    def __init__(self, trajectory_type='circle', max_velocity=0.5, radius=2.0, sim_freq=10.0):
        self.trajectory_type = trajectory_type
        self.max_velocity = max_velocity
        self.radius = radius
        self.sim_freq = sim_freq
        
        self.current_time = 0.0
        self.start_time = time.time()
        self.trajectory_data = []
        self.is_running = False
        
        print(f"✓ 轨迹模拟器已初始化")
        print(f"  类型: {trajectory_type}")
        print(f"  最大速度: {max_velocity} m/s")
        print(f"  半径: {radius} m")
        print(f"  频率: {sim_freq} Hz")
    
    def get_trajectory_point(self, t):
        """获取轨迹点"""
        if self.trajectory_type == 'circle':
            return self.get_circle_trajectory(t)
        elif self.trajectory_type == 'figure8':
            return self.get_figure8_trajectory(t)
        elif self.trajectory_type == 'sine':
            return self.get_sine_trajectory(t)
        elif self.trajectory_type == 'line':
            return self.get_line_trajectory(t)
        else:
            return self.get_circle_trajectory(t)
    
    def get_circle_trajectory(self, t):
        """圆形轨迹"""
        angular_velocity = self.max_velocity / self.radius
        theta = angular_velocity * t
        
        x = self.radius * math.cos(theta)
        y = self.radius * math.sin(theta)
        robot_theta = theta + math.pi / 2
        
        vx = -self.max_velocity * math.sin(theta)
        vy = self.max_velocity * math.cos(theta)
        omega = angular_velocity
        
        return x, y, robot_theta, vx, vy, omega
    
    def get_figure8_trajectory(self, t):
        """8字形轨迹"""
        scale = 1.0 / (2.0 * math.pi)
        phase = (self.max_velocity * t) * scale
        
        x = self.radius * math.sin(phase)
        y = self.radius * math.sin(phase) * math.cos(phase)
        
        dx_dt = self.radius * self.max_velocity * scale * math.cos(phase)
        dy_dt = self.radius * self.max_velocity * scale * (math.cos(2*phase) - math.sin(2*phase))
        
        robot_theta = math.atan2(dy_dt, dx_dt)
        vx = dx_dt
        vy = dy_dt
        omega = 0.2 * math.sin(phase)
        
        return x, y, robot_theta, vx, vy, omega
    
    def get_sine_trajectory(self, t):
        """正弦轨迹"""
        x = self.max_velocity * t
        y = self.radius * math.sin(2 * math.pi * x / (2 * self.radius))
        
        derivative = (2 * math.pi * self.radius * math.cos(2 * math.pi * x / (2 * self.radius))) / (2 * self.radius)
        robot_theta = math.atan(derivative)
        
        vx = self.max_velocity
        vy = self.max_velocity * derivative
        omega = 0.0
        
        return x, y, robot_theta, vx, vy, omega
    
    def get_line_trajectory(self, t):
        """直线轨迹"""
        x = self.max_velocity * t
        y = 0.0
        robot_theta = 0.0
        
        vx = self.max_velocity
        vy = 0.0
        omega = 0.0
        
        return x, y, robot_theta, vx, vy, omega
    
    def simulate_trajectory(self, duration=30.0):
        """模拟轨迹"""
        self.is_running = True
        self.trajectory_data = []
        
        print(f"\n【开始模拟轨迹】")
        print(f"持续时间: {duration} 秒")
        print(f"预计数据点: {int(duration * self.sim_freq)}")
        print("-" * 60)
        
        dt = 1.0 / self.sim_freq
        elapsed = 0.0
        data_point_count = 0
        
        try:
            while elapsed < duration and self.is_running:
                x, y, theta, vx, vy, omega = self.get_trajectory_point(self.current_time)
                
                # 计算速度和角速度
                speed = math.sqrt(vx**2 + vy**2)
                
                data_point = {
                    'time': self.current_time,
                    'x': round(x, 4),
                    'y': round(y, 4),
                    'theta': round(theta, 4),
                    'vx': round(vx, 4),
                    'vy': round(vy, 4),
                    'speed': round(speed, 4),
                    'omega': round(omega, 4)
                }
                
                self.trajectory_data.append(data_point)
                
                # 定期打印信息
                if data_point_count % max(1, int(self.sim_freq)) == 0:
                    self.print_status(data_point, elapsed, duration)
                
                self.current_time += dt
                elapsed += dt
                data_point_count += 1
                
                # 模拟实时执行
                time.sleep(dt * 0.1)  # 加快10倍
        
        except KeyboardInterrupt:
            print("\n\n⚠️  用户中断")
            self.is_running = False
        
        print("-" * 60)
        print(f"✓ 模拟完成！收集 {len(self.trajectory_data)} 个数据点\n")
        
        return self.trajectory_data
    
    def print_status(self, data_point, elapsed, total_duration):
        """打印状态"""
        progress = (elapsed / total_duration) * 100
        print(f"  [{progress:5.1f}%] t={data_point['time']:6.2f}s | "
              f"位置:({data_point['x']:6.3f}, {data_point['y']:6.3f}) | "
              f"速度:{data_point['speed']:5.3f} m/s")
    
    def print_statistics(self):
        """打印统计信息"""
        if not self.trajectory_data:
            print("⚠️  没有轨迹数据")
            return
        
        print("\n" + "=" * 70)
        print("【轨迹统计信息】".center(70))
        print("=" * 70)
        
        # 提取坐标
        positions = [(p['x'], p['y']) for p in self.trajectory_data]
        
        # 位移
        start_pos = positions[0]
        end_pos = positions[-1]
        displacement = math.sqrt((end_pos[0]-start_pos[0])**2 + (end_pos[1]-start_pos[1])**2)
        
        # 总路程
        total_distance = 0.0
        for i in range(1, len(positions)):
            dx = positions[i][0] - positions[i-1][0]
            dy = positions[i][1] - positions[i-1][1]
            total_distance += math.sqrt(dx**2 + dy**2)
        
        # 速度统计
        speeds = [p['speed'] for p in self.trajectory_data]
        avg_speed = sum(speeds) / len(speeds) if speeds else 0
        max_speed = max(speeds) if speeds else 0
        min_speed = min(speeds) if speeds else 0
        
        # 时间
        elapsed_time = self.trajectory_data[-1]['time']
        
        print(f"\n📊 基本信息:")
        print(f"  运行时间:      {elapsed_time:.2f} 秒")
        print(f"  采样频率:      {self.sim_freq:.1f} Hz")
        print(f"  数据点数:      {len(self.trajectory_data)} 个")
        
        print(f"\n📍 位置信息:")
        print(f"  起始位置:      ({start_pos[0]:.4f}, {start_pos[1]:.4f})")
        print(f"  结束位置:      ({end_pos[0]:.4f}, {end_pos[1]:.4f})")
        print(f"  位移:          {displacement:.4f} 米")
        print(f"  总路程:        {total_distance:.4f} 米")
        if displacement > 0:
            efficiency = displacement / total_distance if total_distance > 0 else 0
            print(f"  效率比:        {efficiency:.4f} (位移/路程)")
        
        print(f"\n🚗 速度信息:")
        print(f"  平均速度:      {avg_speed:.4f} m/s")
        print(f"  最大速度:      {max_speed:.4f} m/s")
        print(f"  最小速度:      {min_speed:.4f} m/s")
        
        print("\n" + "=" * 70)
    
    def save_trajectory(self, filename=None):
        """保存轨迹数据到JSON文件"""
        if not self.trajectory_data:
            print("⚠️  没有轨迹数据可保存")
            return None
        
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"trajectory_{self.trajectory_type}_{timestamp}.json"
        
        filepath = Path("/root/ros_ws/trajectories") / filename
        filepath.parent.mkdir(parents=True, exist_ok=True)
        
        trajectory_obj = {
            'metadata': {
                'type': self.trajectory_type,
                'max_velocity': self.max_velocity,
                'radius': self.radius,
                'sim_freq': self.sim_freq,
                'timestamp': datetime.now().isoformat(),
                'duration': self.trajectory_data[-1]['time'],
                'points_count': len(self.trajectory_data)
            },
            'data': self.trajectory_data
        }
        
        with open(filepath, 'w') as f:
            json.dump(trajectory_obj, f, indent=2)
        
        print(f"✓ 轨迹已保存: {filepath}")
        return str(filepath)
    
    def visualize_ascii(self):
        """ASCII可视化轨迹"""
        if not self.trajectory_data:
            print("⚠️  没有轨迹数据")
            return
        
        # 获取边界
        x_coords = [p['x'] for p in self.trajectory_data]
        y_coords = [p['y'] for p in self.trajectory_data]
        
        x_min, x_max = min(x_coords), max(x_coords)
        y_min, y_max = min(y_coords), max(y_coords)
        
        # 添加边距
        margin = 0.5
        x_min -= margin
        x_max += margin
        y_min -= margin
        y_max += margin
        
        # 调整纵横比
        width = 80
        height = 30
        
        x_scale = width / (x_max - x_min)
        y_scale = height / (y_max - y_min)
        
        # 创建画布
        canvas = [['.' for _ in range(width)] for _ in range(height)]
        
        # 绘制轨迹
        for p in self.trajectory_data:
            px = int((p['x'] - x_min) * x_scale)
            py = int((p['y'] - y_min) * y_scale)
            
            if 0 <= px < width and 0 <= py < height:
                canvas[height - 1 - py][px] = '·'
        
        # 标记起点和终点
        start = self.trajectory_data[0]
        end = self.trajectory_data[-1]
        
        sx = int((start['x'] - x_min) * x_scale)
        sy = int((start['y'] - y_min) * y_scale)
        ex = int((end['x'] - x_min) * x_scale)
        ey = int((end['y'] - y_min) * y_scale)
        
        if 0 <= sx < width and 0 <= sy < height:
            canvas[height - 1 - sy][sx] = 'S'
        if 0 <= ex < width and 0 <= ey < height:
            canvas[height - 1 - ey][ex] = 'E'
        
        # 打印
        print("\n【轨迹可视化 (S=起点, E=终点)】")
        print("+" + "-" * width + "+")
        for row in canvas:
            print("|" + "".join(row) + "|")
        print("+" + "-" * width + "+")
        print(f"X: [{x_min:.2f}, {x_max:.2f}]  Y: [{y_min:.2f}, {y_max:.2f}]\n")


def main():
    """主函数"""
    print("\n" + "=" * 70)
    print("【ROS机器人轨迹模拟器 - 独立版本】".center(70))
    print("=" * 70)
    
    print("\n可用的轨迹类型:")
    print("  1. circle   - 圆形轨迹 (默认)")
    print("  2. figure8  - 8字形轨迹")
    print("  3. sine     - 正弦波轨迹")
    print("  4. line     - 直线轨迹")
    
    trajectory_type = 'circle'
    max_velocity = 0.5
    radius = 2.0
    sim_freq = 10.0
    
    # 创建模拟器
    simulator = TrajectorySimulatorStandalone(
        trajectory_type=trajectory_type,
        max_velocity=max_velocity,
        radius=radius,
        sim_freq=sim_freq
    )
    
    # 运行模拟
    print(f"\n开始模拟 '{trajectory_type}' 轨迹...\n")
    simulator.simulate_trajectory(duration=30.0)
    
    # 打印统计
    simulator.print_statistics()
    
    # ASCII可视化
    simulator.visualize_ascii()
    
    # 保存数据
    simulator.save_trajectory()
    
    print("✓ 模拟完成！")
    print("=" * 70 + "\n")


if __name__ == '__main__':
    main()
