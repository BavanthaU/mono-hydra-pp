#pragma once

#include <regex>

#include <rosbag2_transport/play_options.hpp>

namespace ianvs {

struct BagTopicChecker {
  BagTopicChecker(const rosbag2_transport::PlayOptions& options);
  bool passes(const std::string& topic) const;

  std::regex include_regex_;
  std::regex exclude_regex_;
};

}  // namespace ianvs
