#include <tf2_ros/static_transform_broadcaster.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>

#include <rclcpp/rclcpp.hpp>

class TransformFileBroadcaster : public rclcpp::Node {
 public:
  TransformFileBroadcaster()
      : Node("transform_file_broadcaster"),
        broadcaster_(new tf2_ros::StaticTransformBroadcaster(this)) {
    const auto prefix = declare_parameter("prefix", "");
    const auto filepath = declare_parameter("filepath", "");
    if (filepath.empty()) {
      RCLCPP_WARN_STREAM(get_logger(), "No filepath provided!");
      broadcaster_.reset();
      return;
    }

    std::filesystem::path transform_file(filepath);
    if (!std::filesystem::exists(transform_file)) {
      RCLCPP_ERROR_STREAM(get_logger(), "Provided file " << transform_file << " does not exist!");
      broadcaster_.reset();
      return;
    }

    const auto stamp = now();
    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    RCLCPP_INFO_STREAM(get_logger(), "Loading transforms from " << transform_file);
    try {
      const auto node = YAML::LoadFile(transform_file);
      for (const auto& tf : node["frames"]) {
        std::string frame_id = tf["frame_id"].as<std::string>();
        std::string child_id = tf["child_frame_id"].as<std::string>();
        if (!prefix.empty()) {
          frame_id = prefix + frame_id;
          child_id = prefix + child_id;
        }

        auto& msg = transforms.emplace_back();
        msg.header.stamp = stamp;
        msg.header.frame_id = frame_id;
        msg.child_frame_id = child_id;
        msg.transform.translation.x = tf["x"].as<double>();
        msg.transform.translation.y = tf["y"].as<double>();
        msg.transform.translation.z = tf["z"].as<double>();
        msg.transform.rotation.w = tf["qw"].as<double>();
        msg.transform.rotation.x = tf["qx"].as<double>();
        msg.transform.rotation.y = tf["qy"].as<double>();
        msg.transform.rotation.z = tf["qz"].as<double>();
      }
    } catch (std::exception& e) {
      RCLCPP_ERROR_STREAM(get_logger(),
                          "Provided file " << transform_file << " was invalid: " << e.what());
      broadcaster_.reset();
      return;
    }

    if (transforms.empty()) {
      RCLCPP_WARN_STREAM(get_logger(), "No transforms provided!");
      broadcaster_.reset();
      return;
    }

    broadcaster_->sendTransform(transforms);
  }

 private:
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TransformFileBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
