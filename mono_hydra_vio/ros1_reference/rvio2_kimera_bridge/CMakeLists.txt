cmake_minimum_required(VERSION 3.0.2)
project(rvio2_kimera_bridge)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/rgb_to_mono_bridge.py
  scripts/rvio2_path_to_odometry.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
