cmake_minimum_required(VERSION 3.8)
project(mono_hydra)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz ros1_reference
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
