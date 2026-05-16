#include "ianvs/detail/bag_topic_checker.h"

namespace ianvs {
namespace {

inline void regex_append(std::string& regex_string, const std::string& new_regex) {
  regex_string += regex_string.empty() ? new_regex : "|" + new_regex;
}

}  // namespace

BagTopicChecker::BagTopicChecker(const rosbag2_transport::PlayOptions& options) {
  std::string include_str;
  for (const auto& topic : options.topics_to_filter) {
    // TODO(nathan) ideally we would normalize these topics
    regex_append(include_str, topic);
  }

  if (!options.regex_to_filter.empty()) {
    regex_append(include_str, options.regex_to_filter);
  }

  include_str = include_str.empty() ? ".*" : include_str;
  include_regex_ = std::regex(include_str);

  std::string exclude_str;
  for (const auto& topic : options.exclude_topics_to_filter) {
    // TODO(nathan) ideally we would normalize these topics
    regex_append(exclude_str, topic);
  }

  if (!options.exclude_regex_to_filter.empty()) {
    regex_append(exclude_str, options.exclude_regex_to_filter);
  }

  exclude_str = exclude_str.empty() ? "(?!)" : exclude_str;
  exclude_regex_ = std::regex(exclude_str);
}

bool BagTopicChecker::passes(const std::string& topic) const {
  std::smatch match;
  bool should_include = std::regex_match(topic, match, include_regex_);
  bool should_exclude = std::regex_match(topic, match, exclude_regex_);
  return should_include && !should_exclude;
}

}  // namespace ianvs
