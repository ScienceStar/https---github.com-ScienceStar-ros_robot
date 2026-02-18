#!/bin/bash

# ROS Noetic 安装脚本（Ubuntu 22.04 LTS）

set -e

echo "======================================"
echo "开始安装 ROS Noetic"
echo "======================================"

# 1. 设置源
echo "1. 設置 ROS 源..."
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# 2. 添加密钥
echo "2. 添加 ROS 密钥..."
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - || \
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BABE3D 2>/dev/null || true

# 3. 更新包列表
echo "3. 更新包列表..."
sudo apt-get update

# 4. 安装 ROS Noetic 桌面完全版
echo "4. 安装 ROS Noetic..."
sudo apt-get install -y ros-noetic-desktop-full

# 5. 初始化 rosdep
echo "5. 初始化 rosdep..."
sudo rosdep init 2>/dev/null || true
rosdep update

# 6. 设置环境变量
echo "6. 设置环境..."
echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
source ~/.bashrc

# 7. 安装 catkin 工具
echo "7. 安装 catkin 工具..."
sudo apt-get install -y python3-catkin-tools

echo "======================================"
echo "ROS Noetic 安装完成！"
echo "======================================"
echo ""
echo "请运行以下命令激活环境："
echo "  source ~/.bashrc"
echo ""
echo "然后编译项目："
echo "  cd /root/ros_ws && catkin_make"
