#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
轨迹可视化工具 - 显示机器人轨迹的统计信息
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
import math


class TrajectoryVisualizer:
    """轨迹可视化和分析类"""
    
    def __init__(self):
        rospy.init_node('trajectory_visualizer', anonymous=False)
        
        # 订阅者
        self.path_sub = rospy.Subscriber('trajectory/path', Path, self.path_callback)
        self.pose_sub = rospy.Subscriber('trajectory/pose', PoseStamped, self.pose_callback)
        self.velocity_sub = rospy.Subscriber('trajectory/velocity', Twist, self.velocity_callback)
        
        # 数据存储
        self.trajectory_points = []
        self.velocities = []
        self.current_pose = None
        
        self.start_time = rospy.Time.now()
        
        rospy.loginfo("轨迹可视化器已启动")
    
    def path_callback(self, msg):
        """路径消息回调"""
        pass
    
    def pose_callback(self, msg):
        """姿态消息回调"""
        self.current_pose = msg
        if len(self.trajectory_points) < 5000:  # 存储限制
            self.trajectory_points.append([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ])
    
    def velocity_callback(self, msg):
        """速度消息回调"""
        speed = math.sqrt(
            msg.linear.x**2 + 
            msg.linear.y**2 + 
            msg.linear.z**2
        )
        if len(self.velocities) < 5000:
            self.velocities.append(speed)
    
    def print_statistics(self):
        """打印轨迹统计信息"""
        if len(self.trajectory_points) < 2:
            rospy.loginfo("数据不足")
            return
        
        points = np.array(self.trajectory_points)
        
        # 计算统计数据
        displacement = np.linalg.norm(points[-1] - points[0])
        
        # 计算总路程
        distances = np.diff(points, axis=0)
        total_distance = np.sum(np.linalg.norm(distances, axis=1))
        
        # 速度统计
        if self.velocities:
            avg_velocity = np.mean(self.velocities)
            max_velocity = np.max(self.velocities)
        else:
            avg_velocity = 0
            max_velocity = 0
        
        # 时间统计
        elapsed_time = (rospy.Time.now() - self.start_time).to_sec()
        
        rospy.loginfo("=" * 50)
        rospy.loginfo("轨迹统计信息")
        rospy.loginfo("=" * 50)
        rospy.loginfo(f"运行时间: {elapsed_time:.2f} 秒")
        rospy.loginfo(f"轨迹点数: {len(self.trajectory_points)}")
        rospy.loginfo(f"位移: {displacement:.3f} 米")
        rospy.loginfo(f"总路程: {total_distance:.3f} 米")
        rospy.loginfo(f"平均速度: {avg_velocity:.3f} m/s")
        rospy.loginfo(f"最大速度: {max_velocity:.3f} m/s")
        if displacement > 0:
            rospy.loginfo(f"效率比 (位移/路程): {displacement/total_distance:.3f}")
        rospy.loginfo("=" * 50)


def main():
    try:
        visualizer = TrajectoryVisualizer()
        
        # 定期打印统计
        rate = rospy.Rate(0.2)  # 每5秒打印一次
        while not rospy.is_shutdown():
            visualizer.print_statistics()
            rate.sleep()
        
    except KeyboardInterrupt:
        rospy.loginfo("收到中断信号")


if __name__ == '__main__':
    main()
