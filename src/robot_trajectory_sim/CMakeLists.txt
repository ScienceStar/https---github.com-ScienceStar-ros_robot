cmake_minimum_required(VERSION 2.8.3)
project(robot_trajectory_sim)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  tf
)

catkin_package(
)

# Mark Python scripts as executable
catkin_install_python(PROGRAMS
  scripts/trajectory_simulator_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
