#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
机器人轨迹模拟节点
模拟生成和发布机器人的运动轨迹数据
"""

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, Odometry
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float32MultiArray


class TrajectorySimulator:
    """机器人轨迹模拟器类"""
    
    def __init__(self):
        rospy.init_node('trajectory_simulator_node', anonymous=False)
        
        # 获取参数
        self.sim_freq = rospy.get_param('~sim_freq', 10.0)  # 模拟频率(Hz)
        self.trajectory_type = rospy.get_param('~trajectory_type', 'circle')  # 轨迹类型
        self.max_velocity = rospy.get_param('~max_velocity', 0.5)  # 最大速度(m/s)
        self.radius = rospy.get_param('~radius', 2.0)  # 轨迹半径(m)
        
        # 发布器
        self.path_pub = rospy.Publisher('trajectory/path', Path, queue_size=10, latch=True)
        self.pose_pub = rospy.Publisher('trajectory/pose', PoseStamped, queue_size=10)
        self.odom_pub = rospy.Publisher('trajectory/odom', Odometry, queue_size=10)
        self.velocity_pub = rospy.Publisher('trajectory/velocity', Twist, queue_size=10)
        
        # 状态变量
        self.current_time = 0.0
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'world'
        
        # 定时器
        self.timer = rospy.Timer(rospy.Duration(1.0/self.sim_freq), self.timer_callback)
        
        rospy.loginfo(f"轨迹模拟器已启动 - 类型: {self.trajectory_type}, 频率: {self.sim_freq} Hz")
    
    def timer_callback(self, event):
        """定时器回调函数"""
        self.simulate_trajectory()
        self.current_time += 1.0 / self.sim_freq
    
    def get_trajectory_point(self, t):
        """
        根据轨迹类型获取轨迹点
        
        参数:
            t: 时间(秒)
        
        返回:
            x, y, theta, vx, vy, omega
        """
        if self.trajectory_type == 'circle':
            return self.get_circle_trajectory(t)
        elif self.trajectory_type == 'figure8':
            return self.get_figure8_trajectory(t)
        elif self.trajectory_type == 'sine':
            return self.get_sine_trajectory(t)
        elif self.trajectory_type == 'line':
            return self.get_line_trajectory(t)
        else:
            rospy.logwarn(f"未知的轨迹类型: {self.trajectory_type}")
            return self.get_circle_trajectory(t)
    
    def get_circle_trajectory(self, t):
        """圆形轨迹"""
        angular_velocity = self.max_velocity / self.radius  # 角速度
        theta = angular_velocity * t  # 当前角度
        
        x = self.radius * math.cos(theta)
        y = self.radius * math.sin(theta)
        
        # 机器人朝向 - 切向
        robot_theta = theta + math.pi / 2
        
        # 速度
        vx = -self.max_velocity * math.sin(theta)
        vy = self.max_velocity * math.cos(theta)
        omega = angular_velocity
        
        return x, y, robot_theta, vx, vy, omega
    
    def get_figure8_trajectory(self, t):
        """8字形轨迹"""
        scale = 1.0 / (2.0 * math.pi)
        phase = (self.max_velocity * t) * scale
        
        # 使用参数化方程
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
    
    def simulate_trajectory(self):
        """模拟轨迹并发布消息"""
        x, y, theta, vx, vy, omega = self.get_trajectory_point(self.current_time)
        
        # 创建姿态消息
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = 'world'
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = 0.0
        
        # 转换欧拉角为四元数
        quat = quaternion_from_euler(0, 0, theta)
        pose_stamped.pose.orientation.x = quat[0]
        pose_stamped.pose.orientation.y = quat[1]
        pose_stamped.pose.orientation.z = quat[2]
        pose_stamped.pose.orientation.w = quat[3]
        
        # 发布姿态
        self.pose_pub.publish(pose_stamped)
        
        # 更新路径
        self.path_msg.header.stamp = rospy.Time.now()
        self.path_msg.poses.append(pose_stamped)
        if len(self.path_msg.poses) > 1000:  # 限制路径长度
            self.path_msg.poses.pop(0)
        self.path_pub.publish(self.path_msg)
        
        # 创建里程计消息
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'base_link'
        odom.pose.pose = pose_stamped.pose
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega
        self.odom_pub.publish(odom)
        
        # 创建速度消息
        velocity = Twist()
        velocity.linear.x = vx
        velocity.linear.y = vy
        velocity.linear.z = 0.0
        velocity.angular.z = omega
        self.velocity_pub.publish(velocity)
    
    def shutdown(self):
        """关闭函数"""
        rospy.loginfo("轨迹模拟器已关闭")


def main():
    try:
        simulator = TrajectorySimulator()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("收到中断信号")
    except Exception as e:
        rospy.logerr(f"发生错误: {e}")


if __name__ == '__main__':
    main()
