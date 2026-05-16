#pragma once
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
namespace sensor_msgs
{
using Image = msg::Image;
using ImageConstPtr = msg::Image::ConstSharedPtr;
namespace image_encodings = ::sensor_msgs::image_encodings;
}
