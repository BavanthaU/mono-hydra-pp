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

# RVIO2 variant with SuperPoint guided feature tracking (src/rvio2_sp). The
# classical rvio2_core/rvio2_mono_node above are left untouched.
option(MONO_HYDRA_BUILD_RVIO2_SP "Build rvio2_sp_mono_node (RVIO2 with SuperPoint tracking)" ON)
if(MONO_HYDRA_BUILD_RVIO2_SP)
  set(RVIO2_SP_ORT_INCLUDE_DIR
    ${CMAKE_CURRENT_LIST_DIR}/ros1_reference/mono_hydra_vio/third_party/onnxruntime-linux-x64-gpu-1.18.0/include
    CACHE PATH "ONNX Runtime C/C++ headers for rvio2_sp")
  set(RVIO2_SP_ORT_LIB_DIR "" CACHE PATH
    "Directory with libonnxruntime.so.1.* (default: pip onnxruntime-gpu, else the bundled 1.18 runtime)")
  set(RVIO2_SP_CUDA_LIB_DIRS "" CACHE STRING
    "CUDA/cuDNN library directories added to the rvio2_sp RPATH (default: pip nvidia-* packages)")

  find_package(Python3 COMPONENTS Interpreter)

  if(NOT RVIO2_SP_ORT_LIB_DIR AND Python3_Interpreter_FOUND)
    execute_process(
      COMMAND ${Python3_EXECUTABLE} -c
        "import onnxruntime, os; print(os.path.join(os.path.dirname(onnxruntime.__file__), 'capi'))"
      OUTPUT_VARIABLE _rvio2_sp_ort_dir
      OUTPUT_STRIP_TRAILING_WHITESPACE
      RESULT_VARIABLE _rvio2_sp_ort_result
      ERROR_QUIET)
    if(_rvio2_sp_ort_result EQUAL 0 AND EXISTS "${_rvio2_sp_ort_dir}")
      set(RVIO2_SP_ORT_LIB_DIR "${_rvio2_sp_ort_dir}")
    endif()
  endif()
  if(NOT RVIO2_SP_ORT_LIB_DIR)
    set(RVIO2_SP_ORT_LIB_DIR
      ${CMAKE_CURRENT_LIST_DIR}/ros1_reference/mono_hydra_vio/third_party/onnxruntime-linux-x64-gpu-1.18.0/lib)
  endif()

  file(GLOB _rvio2_sp_ort_libs "${RVIO2_SP_ORT_LIB_DIR}/libonnxruntime.so.1.*")
  if(NOT _rvio2_sp_ort_libs)
    message(FATAL_ERROR "rvio2_sp: no libonnxruntime.so.1.* in ${RVIO2_SP_ORT_LIB_DIR}")
  endif()
  list(GET _rvio2_sp_ort_libs 0 _rvio2_sp_ort_lib)

  if(NOT RVIO2_SP_CUDA_LIB_DIRS AND Python3_Interpreter_FOUND)
    execute_process(
      COMMAND ${Python3_EXECUTABLE} -c
        "import glob, os, importlib.util as u; s = u.find_spec('nvidia'); print(';'.join(sorted(d for p in (s.submodule_search_locations if s else []) for d in glob.glob(os.path.join(p, '*', 'lib')))))"
      OUTPUT_VARIABLE RVIO2_SP_CUDA_LIB_DIRS
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET)
  endif()

  # The pip wheel ships libonnxruntime.so.1.x without the soname link, and ORT
  # loads its execution providers from the directory it was loaded from, so
  # stage soname and provider links in one private directory.
  set(_rvio2_sp_ort_stage ${CMAKE_CURRENT_BINARY_DIR}/rvio2_sp_ort)
  file(MAKE_DIRECTORY ${_rvio2_sp_ort_stage})
  file(CREATE_LINK ${_rvio2_sp_ort_lib} ${_rvio2_sp_ort_stage}/libonnxruntime.so.1 SYMBOLIC)
  file(GLOB _rvio2_sp_ort_providers "${RVIO2_SP_ORT_LIB_DIR}/libonnxruntime_providers_*.so")
  foreach(_provider ${_rvio2_sp_ort_providers})
    get_filename_component(_provider_name ${_provider} NAME)
    file(CREATE_LINK ${_provider} ${_rvio2_sp_ort_stage}/${_provider_name} SYMBOLIC)
  endforeach()
  message(STATUS "rvio2_sp: ONNX Runtime ${_rvio2_sp_ort_lib}")
  message(STATUS "rvio2_sp: CUDA library dirs ${RVIO2_SP_CUDA_LIB_DIRS}")

  add_library(rvio2_sp_core SHARED
    src/rvio2_sp/System.cc
    src/rvio2_sp/Ransac.cc
    src/rvio2_sp/Tracker.cc
    src/rvio2_sp/Updater.cc
    src/rvio2_sp/Propagator.cc
    src/rvio2_sp/InputBuffer.cc
    src/rvio2_sp/Feature.cc
    src/rvio2_sp/FeatureDetector.cc
    src/rvio2_sp/SuperPoint.cc
  )
  ament_target_dependencies(rvio2_sp_core
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
  target_include_directories(rvio2_sp_core SYSTEM PRIVATE ${RVIO2_SP_ORT_INCLUDE_DIR})
  target_link_libraries(rvio2_sp_core ${OpenCV_LIBS} ${_rvio2_sp_ort_stage}/libonnxruntime.so.1)

  add_executable(rvio2_sp_mono_node src/rvio2_sp_mono_ros2.cc)
  ament_target_dependencies(rvio2_sp_mono_node
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
  target_link_libraries(rvio2_sp_mono_node rvio2_sp_core ${OpenCV_LIBS})

  # DT_RPATH (not RUNPATH) so it also applies to the CUDA provider that ORT
  # dlopens; no LD_LIBRARY_PATH setup is needed at runtime.
  set(_rvio2_sp_install_ort ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/rvio2_sp_ort)
  foreach(_target rvio2_sp_core rvio2_sp_mono_node)
    target_link_options(${_target} PRIVATE -Wl,--disable-new-dtags)
    set_target_properties(${_target} PROPERTIES
      BUILD_RPATH "${_rvio2_sp_ort_stage};${CMAKE_CURRENT_BINARY_DIR};${RVIO2_SP_CUDA_LIB_DIRS}"
      INSTALL_RPATH "${_rvio2_sp_install_ort};${CMAKE_INSTALL_PREFIX}/lib;${RVIO2_SP_CUDA_LIB_DIRS}")
  endforeach()

  install(DIRECTORY ${_rvio2_sp_ort_stage}/ DESTINATION lib/${PROJECT_NAME}/rvio2_sp_ort)
  install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
endif()

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
if(TARGET rvio2_sp_mono_node)
  list(APPEND _mono_hydra_vio_targets rvio2_sp_core rvio2_sp_mono_node)
endif()
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
