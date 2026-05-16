cmake_minimum_required(VERSION 3.8)
project(mono_hydra_vio)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(DBoW2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(KimeraRPGO REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(opengv REQUIRED)
find_package(pose_graph_tools_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

if(NOT TARGET DBoW2::DBoW2)
  add_library(DBoW2::DBoW2 INTERFACE IMPORTED)
  set_target_properties(DBoW2::DBoW2 PROPERTIES
    INTERFACE_LINK_LIBRARIES "${DBoW2_LIBRARIES}"
    INTERFACE_INCLUDE_DIRECTORIES "${DBoW2_INCLUDE_DIRS}")
endif()

option(MONO_HYDRA_BUILD_KIMERA_VIO_CORE
  "Build the benchmarked Kimera VIO core used by the ROS 1 Mono Hydra wrapper"
  ON)
if(MONO_HYDRA_BUILD_KIMERA_VIO_CORE)
  set(KIMERA_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(KIMERA_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(KIMERA_VERIFY_GTSAM_CONFIG OFF CACHE BOOL "" FORCE)
  set(EXPORT_KIMERA OFF CACHE BOOL "" FORCE)
  add_subdirectory(ros1_reference/mono_hydra_vio)
endif()

include_directories(
  include
  src
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(rvio2_core SHARED
  src/rvio2/System.cc
  src/rvio2/Ransac.cc
  src/rvio2/Tracker.cc
  src/rvio2/Updater.cc
  src/rvio2/Propagator.cc
  src/rvio2/InputBuffer.cc
  src/rvio2/Feature.cc
  src/rvio2/FeatureDetector.cc
)
ament_target_dependencies(rvio2_core
  ament_index_cpp
  cv_bridge
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2_ros
  visualization_msgs
)
target_link_libraries(rvio2_core ${OpenCV_LIBS})

add_executable(rvio2_mono_node src/rvio2_mono_ros2.cc)
ament_target_dependencies(rvio2_mono_node
  ament_index_cpp
  cv_bridge
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2_ros
  visualization_msgs
)
target_link_libraries(rvio2_mono_node rvio2_core ${OpenCV_LIBS})

if(MONO_HYDRA_BUILD_KIMERA_VIO_CORE)
  set(ONNXRUNTIME_THIRDPARTY_DIR
    ${CMAKE_CURRENT_LIST_DIR}/ros1_reference/mono_hydra_vio/third_party/onnxruntime-linux-x64-gpu-1.18.0)
  add_executable(mono_hydra_vio_ros_node src/kimera_vio_ros2_node.cpp)
  ament_target_dependencies(mono_hydra_vio_ros_node
    cv_bridge
    geometry_msgs
    message_filters
    nav_msgs
    pose_graph_tools_msgs
    rclcpp
    sensor_msgs
    std_msgs
    tf2_ros
  )
  target_include_directories(mono_hydra_vio_ros_node PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/ros1_reference/mono_hydra_vio/include
  )
  target_link_libraries(mono_hydra_vio_ros_node
    kimera_vio
    opengv
    DBoW2::DBoW2
    KimeraRPGO
    ${ONNXRUNTIME_THIRDPARTY_DIR}/lib/libonnxruntime.so
    ${OpenCV_LIBS}
  )
endif()

ament_python_install_package(${PROJECT_NAME})

set(_mono_hydra_vio_targets rvio2_core rvio2_mono_node)
if(TARGET mono_hydra_vio_ros_node)
  list(APPEND _mono_hydra_vio_targets mono_hydra_vio_ros_node)
endif()

install(TARGETS ${_mono_hydra_vio_targets}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS mono_hydra_vio/odom_adapter_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME odom_adapter_node
)

install(PROGRAMS mono_hydra_vio/rgb_to_mono_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rgb_to_mono_node
)

install(PROGRAMS mono_hydra_vio/vio_feature_interface_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME vio_feature_interface_node
)

install(PROGRAMS mono_hydra_vio/kimera_pose_graph_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME kimera_pose_graph_bridge_node
)

install(PROGRAMS scripts/download_orb_vocabulary.sh
  DESTINATION lib/${PROJECT_NAME}
  RENAME download_orb_vocabulary
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY params rviz vocabulary
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
