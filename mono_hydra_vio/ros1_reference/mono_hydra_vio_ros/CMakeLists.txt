cmake_minimum_required(VERSION 2.8.3)
project(mono_hydra_vio_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin_simple REQUIRED)

### System Dependencies
find_package(Boost REQUIRED COMPONENTS
  serialization system filesystem thread date_time regex timer chrono)
if(NOT TARGET Boost::boost)
  add_library(Boost::boost INTERFACE IMPORTED)
  set_target_properties(Boost::boost PROPERTIES
  INTERFACE_LINK_LIBRARIES "${Boost_LIBRARIES}"
  INTERFACE_INCLUDE_DIRECTORIES "${Boost_INCLUDE_DIRS}")
endif()

catkin_simple()

find_package(mono_hydra_vio REQUIRED)

cs_add_library(${PROJECT_NAME}
  # Include files are just here for IDEs
  include/mono_hydra_vio_ros/KimeraVioRos.h
  include/mono_hydra_vio_ros/RosBagDataProvider.h
  include/mono_hydra_vio_ros/RosDataProviderInterface.h
  include/mono_hydra_vio_ros/RosDisplay.h
  include/mono_hydra_vio_ros/RosOnlineDataProvider.h
  include/mono_hydra_vio_ros/RosPublishers.h
  include/mono_hydra_vio_ros/RosVisualizer.h
  include/mono_hydra_vio_ros/utils/UtilsRos.h
  include/mono_hydra_vio_ros/utils/CsvPublisher.h
  src/KimeraVioRos.cpp
  src/RosDataProviderInterface.cpp
  src/RosBagDataProvider.cpp
  src/RosOnlineDataProvider.cpp
  src/RosDisplay.cpp
  src/RosVisualizer.cpp
  src/RosLoopClosureVisualizer.cpp
  src/LcdRegistrationServer.cpp
  src/utils/UtilsRos.cpp
  src/utils/CsvPublisher.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC mono_hydra_vio::mono_hydra_vio)

set(ONNXRUNTIME_THIRDPARTY_DIR
    ${PROJECT_SOURCE_DIR}/../mono_hydra_vio/third_party/onnxruntime-linux-x64-gpu-1.18.0)

if(NOT TARGET onnxruntime::onnxruntime)
  add_library(onnxruntime::onnxruntime SHARED IMPORTED)
  set_target_properties(onnxruntime::onnxruntime PROPERTIES
      IMPORTED_LOCATION ${ONNXRUNTIME_THIRDPARTY_DIR}/lib/libonnxruntime.so
      INTERFACE_INCLUDE_DIRECTORIES ${ONNXRUNTIME_THIRDPARTY_DIR}/include)
endif()

target_link_libraries(${PROJECT_NAME} PUBLIC onnxruntime::onnxruntime)

cs_add_executable(${PROJECT_NAME}_node
  src/KimeraVioRosNode.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  mono_hydra_vio::mono_hydra_vio
  onnxruntime::onnxruntime)

cs_add_executable(csv_publisher_node
  src/utils/CsvPublisherNode.cpp
)
target_link_libraries(csv_publisher_node
  ${PROJECT_NAME}
  mono_hydra_vio::mono_hydra_vio
  onnxruntime::onnxruntime)

catkin_add_gtest(testKimeraVioRos
  test/testKimeraVioRos.cpp
)
target_link_libraries(testKimeraVioRos ${PROJECT_NAME})

cs_install()

cs_export()
