#include "ianvs/detail/frame_remapper.h"

#include <rclcpp/logging.hpp>

namespace ianvs {
namespace {

inline std::string frame_name(const std::string& dest, const std::string& src) {
  return "'" + dest + "_T_" + src + "'";
}

}  // namespace

using detail::StringTransform;
using SType = StringTransform::Type;

FrameRemapper::TFRegex::TFRegex(const std::string& arg) {
  const auto pos = arg.find(":");
  if (pos == std::string::npos) {
    parent = std::regex(arg);
    return;
  }

  parent = std::regex(arg.substr(0, pos));
  child = std::regex(arg.substr(pos + 1));
}

bool FrameRemapper::TFRegex::matches(const std::string& parent_id,
                                     const std::string& child_id) const {
  std::smatch match;
  const bool parent_matches = std::regex_match(parent_id, match, parent);
  if (!child) {
    // simple filters apply to either endpoint of the edge
    return parent_matches || std::regex_match(child_id, match, parent);
  }

  // joint filters require both endpoints to match
  return parent_matches && std::regex_match(child_id, match, *child);
}

FrameRemapper::FrameRemapper(const Config& config, const rclcpp::Logger* logger) : config(config) {
  for (const auto& filter : config.filter) {
    filter_.push_back(TFRegex(filter));
  }

  for (const auto& keep : config.keep) {
    keep_.push_back(TFRegex(keep));
  }

  if (!config.prefix.empty()) {
    transforms_.push_back(StringTransform::from_arg(SType::Prefix, config.prefix, logger));
  }

  for (const auto& arg : config.substitutions) {
    transforms_.push_back(StringTransform::from_arg(SType::Substitute, arg, logger));
  }
}

void FrameRemapper::updatePoseMap(const Msg& msg,
                                  PoseMap& pose_map,
                                  const rclcpp::Logger* logger) const {
  for (auto& tf : msg.transforms) {
    auto parent_id = tf.header.frame_id;
    auto child_id = tf.child_frame_id;
    if (shouldFilter(parent_id, child_id) || !shouldKeep(parent_id, child_id)) {
      continue;
    }

    parent_id = remapFrame(parent_id);
    child_id = remapFrame(child_id);
    if (parent_id.empty() || child_id.empty()) {
      if (logger) {
        const auto orig_name = frame_name(tf.header.frame_id, tf.child_frame_id);
        RCLCPP_INFO_STREAM(*logger, "Substitutions resulted in empty frame_id for " << orig_name);
      }

      continue;
    }

    auto parent = pose_map.find(parent_id);
    if (parent == pose_map.end()) {
      parent = pose_map.emplace(parent_id, std::map<std::string, Transform>()).first;
    }

    auto& children = parent->second;
    auto child = children.find(child_id);
    if (child == children.end()) {
      auto iter = children.emplace(child_id, tf).first;
      iter->second.header.frame_id = parent_id;
      iter->second.child_frame_id = child_id;
      continue;
    }

    if (logger) {
      const auto remapped_name = frame_name(parent_id, child_id);
      RCLCPP_INFO_STREAM(*logger, "Dropping repeated tf " << remapped_name);
    }
  }
}

bool FrameRemapper::shouldKeep(const std::string& parent, const std::string& child) const {
  if (keep_.empty()) {
    return true;
  }

  for (const auto& keep : keep_) {
    if (keep.matches(parent, child)) {
      return true;
    }
  }

  return false;
}

bool FrameRemapper::shouldFilter(const std::string& parent, const std::string& child) const {
  if (filter_.empty()) {
    return false;
  }

  for (const auto& filter : filter_) {
    if (filter.matches(parent, child)) {
      return true;
    }
  }

  return false;
}

std::string FrameRemapper::remapFrame(const std::string& frame_id) const {
  std::string result = frame_id;
  for (const auto& transform : transforms_) {
    result = transform.apply(result);
    if (result.empty()) {
      return result;
    }
  }

  return result;
}

}  // namespace ianvs
