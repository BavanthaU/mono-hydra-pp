cmake_minimum_required(VERSION 3.8)
project(mono_hydra)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz ros1_reference
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_itc_launch_defaults test/test_itc_launch_defaults.py)
endif()

ament_package()
