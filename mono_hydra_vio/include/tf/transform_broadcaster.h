#pragma once

#include <memory>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "ros/ros.h"

namespace tf
{
class TransformBroadcaster
{
public:
  TransformBroadcaster()
  {
    if (ros::global_node()) {
      broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(ros::global_node());
    }
  }
  void sendTransform(const geometry_msgs::msg::TransformStamped& transform)
  {
    if (broadcaster_) {
      broadcaster_->sendTransform(transform);
    }
  }

private:
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};
}
