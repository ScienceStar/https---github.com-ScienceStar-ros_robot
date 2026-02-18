# 机器人轨迹模拟工程 (Robot Trajectory Simulation)

这是一个ROS机器人轨迹模拟工程，用于生成和可视化各种机器人运动轨迹。

## 功能特性

- **多种轨迹类型支持**：
  - `circle`: 圆形轨迹
  - `figure8`: 8字形轨迹
  - `sine`: 正弦波轨迹
  - `line`: 直线轨迹

- **完整的ROS消息发布**：
  - 姿态 (`PoseStamped`)
  - 里程计 (`Odometry`)
  - 速度 (`Twist`)
  - 路径 (`Path`)

- **RViz可视化**：预配置的RViz配置文件用于轨迹可视化

- **轨迹分析工具**：包含轨迹统计和分析节点

## 项目结构

```
robot_trajectory_sim/
├── CMakeLists.txt                 # 构建配置
├── package.xml                    # ROS包配置
├── launch/
│   └── trajectory_sim.launch      # 启动文件
├── config/
│   ├── trajectory.rviz            # RViz配置
│   └── trajectory_params.yaml     # 参数配置
└── scripts/
    ├── trajectory_simulator_node.py   # 主轨迹模拟节点
    └── trajectory_visualizer.py       # 轨迹分析可视化工具
```

## 快速开始

### 1. 编译工程

```bash
cd /root/ros_ws
catkin_make
source devel/setup.bash
```

### 2. 运行轨迹模拟

```bash
roslaunch robot_trajectory_sim trajectory_sim.launch
```

这会启动：
- 轨迹模拟节点（发布轨迹数据）
- RViz可视化工具（显示轨迹）

### 3. 运行轨迹分析工具（可选）

在另一个终端：

```bash
rosrun robot_trajectory_sim trajectory_visualizer.py
```

## 参数配置

### launch文件参数

可在 `launch/trajectory_sim.launch` 中修改以下参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `sim_freq` | 20.0 Hz | 模拟频率 |
| `trajectory_type` | circle | 轨迹类型 |
| `max_velocity` | 0.5 m/s | 最大速度 |
| `radius` | 2.0 m | 轨迹半径 |

### 使用不同轨迹

通过启动时传递参数：

```bash
# 圆形轨迹
roslaunch robot_trajectory_sim trajectory_sim.launch trajectory_type:=circle

# 8字形轨迹
roslaunch robot_trajectory_sim trajectory_sim.launch trajectory_type:=figure8

# 正弦波轨迹
roslaunch robot_trajectory_sim trajectory_sim.launch trajectory_type:=sine

# 直线轨迹
roslaunch robot_trajectory_sim trajectory_sim.launch trajectory_type:=line
```

## 发布的话题

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/trajectory/pose` | geometry_msgs/PoseStamped | 机器人当前姿态 |
| `/trajectory/odom` | nav_msgs/Odometry | 机器人里程计 |
| `/trajectory/path` | nav_msgs/Path | 完整的轨迹路径 |
| `/trajectory/velocity` | geometry_msgs/Twist | 机器人当前速度 |

## 使用示例

### 启动基础模拟（圆形轨迹）

```bash
# 终端1：编译并启动模拟
cd /root/ros_ws
catkin_make
source devel/setup.bash
roslaunch robot_trajectory_sim trajectory_sim.launch

# 终端2：监控数据
rostopic echo /trajectory/pose

# 终端3：可视化分析
rosrun robot_trajectory_sim trajectory_visualizer.py
```

### 切换轨迹类型

```bash
# 启动8字形轨迹，半径为3米，速度0.8 m/s
roslaunch robot_trajectory_sim trajectory_sim.launch \
  trajectory_type:=figure8 \
  radius:=3.0 \
  max_velocity:=0.8
```

## 查看轨迹数据

在RViz中，您可以看到：
- **绿色线条**：完整的轨迹路径
- **红色坐标轴**：机器人当前位置和方向
- **网格背景**：参考坐标系

## 轨迹统计信息

轨迹分析工具会定期输出：
- 运行时间
- 轨迹点数
- 机器人位移
- 总路程
- 平均/最大速度
- 效率比（位移/路程）

## 扩展功能

可以在 `trajectory_simulator_node.py` 中添加新的轨迹类型：

1. 在 `TrajectorySimulator` 类中添加新的轨迹方法
2. 在 `get_trajectory_point()` 方法中注册新类型
3. 更新launch/config文件中的参数

例如，添加方形轨迹：

```python
def get_square_trajectory(self, t):
    """方形轨迹"""
    # 实现方形轨迹逻辑
    pass
```

## 常见问题

**Q: 如何修改轨迹的速度？**
A: 修改launch文件中的 `max_velocity` 参数。

**Q: 如何在Python中订阅轨迹数据？**
A: 可以订阅 `/trajectory/pose` 或 `/trajectory/odom` 话题。

**Q: 轨迹显示不出来？**
A: 1. 检查RViz的Fixed Frame是否设置为 "world"
   2. 检查对应话题是否在RViz中被启用
   3. 查看 `rosnode list` 确认节点已启动

## 许可证

BSD

## 维护者

Robot Simulation Team
