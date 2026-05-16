/* @file   KimeraVioRosNode.cpp
 * @brief  ROS Node for mono_hydra_vio
 * @author Antoni Rosinol
 */

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <ros/ros.h>

#include "mono_hydra_vio_ros/KimeraVioRos.h"

int main(int argc, char* argv[]) {
  // Initialize Google's flags library.
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);

  // Initialize ROS node.
  ros::init(argc, argv, "mono_hydra_vio");

  // Initialize VIO pipeline and ROS data provider.
  VIO::KimeraVioRos mono_hydra_vio_ros;

  return mono_hydra_vio_ros.runKimeraVio() ? EXIT_SUCCESS : EXIT_FAILURE;
}
