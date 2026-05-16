#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <rclcpp/serialization.hpp>

#include "ianvs/app/rosbag_play_plugins.h"
#include "ianvs/detail/bag_topic_checker.h"
#include "ianvs/detail/frame_remapper.h"

namespace ianvs {

using Transform = geometry_msgs::msg::TransformStamped;
using tf2_msgs::msg::TFMessage;

using PoseMap = FrameRemapper::PoseMap;
using ArgVec = std::vector<std::string>;

class TFPlugin : public RosbagPlayPlugin {
 public:
  struct Config : FrameRemapper::Config {
    bool filter_dynamic = false;
  } config;

  TFPlugin();

  void init(std::shared_ptr<rclcpp::Node> node) override;
  void add_options(CLI::App& app) override;
  void on_start(rosbag2_cpp::Reader& reader,
                rosbag2_transport::PlayOptions& options,
                const rclcpp::Logger* logger = nullptr) override;
  void on_stop() override {}

  const rclcpp::Serialization<TFMessage> serialization;

 private:
  void callback(TFMessage::UniquePtr msg);
  void publishStaticTFs(const PoseMap& pose_map);

  std::string tf_topic_;
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<FrameRemapper> remapper_;
  rclcpp::Subscription<TFMessage>::SharedPtr sub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_broadcaster_;
};

TFPlugin::TFPlugin() {}

void TFPlugin::init(std::shared_ptr<rclcpp::Node> node) {
  node_ = node;
  tf_topic_ = node_->get_node_topics_interface()->resolve_topic_name("~/_tf");
}

void TFPlugin::add_options(CLI::App& app) {
  const auto group_name = "TF Options";
  app.add_option("-p,--prefix-frames", config.prefix)
      ->take_last()
      ->description("prefix to apply to ALL frames")
      ->group(group_name);
  app.add_option("-f,--filter-frames", config.filter)
      ->description("optional regex filter to drop frames (applied before prefix)")
      ->group(group_name);
  app.add_option("-k,--keep-frames", config.keep)
      ->description("optional regex filter to keep frames (applied before prefix)")
      ->group(group_name);
  app.add_option("-s,--frame-substitution", config.substitutions)
      ->description("apply substitution to frames (match and substituion are separated by :)")
      ->group(group_name);
  app.add_flag(
         "--filter-tf", config.filter_dynamic, "enable filtering /tf in addition to /tf_static")
      ->group(group_name);
  app.footer(R"(
Both --filter-frames and --keep-frames support two match modes for a given
TF. They can either be provided with a single regex that can match either
the parent or the child frame OR they can be provided with two regexes in
the form <PARENT_REGEX>:<CHILD_REGEX>, which requires that the parent frame
matches the first regex and the child frame matches the second regex. This
precludes the use of non-captured groups in the regex
)");
}

void TFPlugin::on_start(rosbag2_cpp::Reader& reader,
                        rosbag2_transport::PlayOptions& options,
                        const rclcpp::Logger* logger) {
  remapper_ = std::make_unique<FrameRemapper>(config, logger);

  BagTopicChecker checker(options);
  if (config.filter_dynamic && checker.passes("/tf")) {
    options.topic_remapping_options.push_back("--remap");
    options.topic_remapping_options.push_back("/tf:=" + tf_topic_);
    dynamic_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    sub_ = node_->create_subscription<TFMessage>(
        tf_topic_,
        tf2_ros::DynamicListenerQoS(),
        std::bind(&TFPlugin::callback, this, std::placeholders::_1));
  }

  std::set<std::string> excluded(options.exclude_topics_to_filter.begin(),
                                 options.exclude_topics_to_filter.end());
  if (!checker.passes("/tf_static")) {
    return;
  }

  broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_);
  options.exclude_topics_to_filter.push_back("/tf_static");

  PoseMap pose_map;
  rosbag2_storage::StorageFilter filter;
  filter.topics = {"/tf_static"};
  reader.set_filter(filter);
  while (reader.has_next()) {
    const auto msg = reader.read_next();
    rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
    auto tf_msg = std::make_shared<TFMessage>();
    serialization.deserialize_message(&serialized_msg, tf_msg.get());
    remapper_->updatePoseMap(*tf_msg, pose_map, logger);
  }

  publishStaticTFs(pose_map);
}

void TFPlugin::callback(TFMessage::UniquePtr msg) {
  if (!remapper_ || !dynamic_broadcaster_) {
    return;
  }

  PoseMap pose_map;
  remapper_->updatePoseMap(*msg, pose_map);
  if (pose_map.empty()) {
    return;
  }

  std::vector<Transform> poses;
  for (const auto& [parent, children] : pose_map) {
    for (const auto& [child, pose] : children) {
      poses.push_back(pose);
    }
  }

  dynamic_broadcaster_->sendTransform(poses);
}

void TFPlugin::publishStaticTFs(const PoseMap& pose_map) {
  std::vector<Transform> poses;
  for (const auto& [parent, children] : pose_map) {
    for (const auto& [child, pose] : children) {
      poses.push_back(pose);
    }
  }

  if (poses.empty() || !broadcaster_) {
    return;
  }

  broadcaster_->sendTransform(poses);
}

}  // namespace ianvs

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ianvs::TFPlugin, ianvs::RosbagPlayPlugin)
