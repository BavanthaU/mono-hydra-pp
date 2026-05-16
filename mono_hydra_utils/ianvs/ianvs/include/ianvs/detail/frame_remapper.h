#pragma once
#include <optional>

#include <tf2_msgs/msg/tf_message.hpp>

#include "ianvs/detail/string_transforms.h"

namespace ianvs {

struct FrameRemapper {
 public:
  using Transform = geometry_msgs::msg::TransformStamped;
  using Msg = tf2_msgs::msg::TFMessage;
  using PoseMap = std::map<std::string, std::map<std::string, Transform>>;

  struct TFRegex {
    explicit TFRegex(const std::string& arg);

    bool matches(const std::string& parent_id, const std::string& child_id) const;

    std::regex parent;
    std::optional<std::regex> child;
  };

  struct Config {
    std::string prefix;
    std::vector<std::string> filter;
    std::vector<std::string> keep;
    std::vector<std::string> substitutions;
  } const config;

  explicit FrameRemapper(const Config& config, const rclcpp::Logger* logger = nullptr);

  void updatePoseMap(const Msg& msg,
                     PoseMap& pose_map,
                     const rclcpp::Logger* logger = nullptr) const;

 private:
  std::string remapFrame(const std::string& frame_id) const;

  bool shouldKeep(const std::string& parent, const std::string& child) const;

  bool shouldFilter(const std::string& parent, const std::string& child) const;

  std::vector<detail::StringTransform> transforms_;
  std::vector<TFRegex> keep_;
  std::vector<TFRegex> filter_;
};

}  // namespace ianvs
