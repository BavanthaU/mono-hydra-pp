cmake_minimum_required(VERSION 3.8)
project(mono_hydra_utils)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kimera_pgmo_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_executable(mesh_marker_node src/mesh_marker_node.cpp)
ament_target_dependencies(mesh_marker_node
  geometry_msgs
  kimera_pgmo_msgs
  rclcpp
  std_msgs
  visualization_msgs
)

install(TARGETS mesh_marker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  mono_hydra_utils/mesh_marker_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mesh_marker_node_py
)

install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
