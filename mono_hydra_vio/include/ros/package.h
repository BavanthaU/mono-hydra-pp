#pragma once

#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace ros::package
{
inline std::string getPath(const std::string& package_name)
{
  try {
    return ament_index_cpp::get_package_share_directory(package_name);
  } catch (...) {
    return ".";
  }
}
}
