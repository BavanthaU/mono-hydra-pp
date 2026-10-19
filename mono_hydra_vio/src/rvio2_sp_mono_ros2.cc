#include <unistd.h>

#include <fstream>
#include <map>
#include <memory>
#include <regex>
#include <sstream>
#include <string>

#include <Eigen/Core>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "ros/ros.h"
#include "rvio2_sp/System.h"

// RVIO2 with SuperPoint guided feature tracking. Same topics and interface as
// rvio2_mono_node; the tracker switches live in the config file and can be
// overridden per launch through the parameters declared below.
class Rvio2SpMonoRos2 : public rclcpp::Node
{
public:
  Rvio2SpMonoRos2() : Node("rvio2_sp_mono_node")
  {
    image_topic_ = declare_parameter<std::string>("image_topic", "/rvio2_bridge/cam0/image_raw");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/camera/imu");
    config_path_ = declare_parameter<std::string>("config_path", "");

    // Overrides: -1 / "" keep the config file value
    int_overrides_ = {
      {"tracker_method", "Tracker.Method"},
      {"detector", "Tracker.Detector"},
      {"klt_gyro_prediction", "Tracker.KltGyroPrediction"},
      {"superpoint_use_gpu", "SuperPoint.UseGpu"},
      {"gyro_prediction", "SPTrack.GyroPrediction"},
      {"klt_fallback", "SPTrack.KltFallback"},
      {"refine", "SPTrack.Refine"},
      {"update_descriptor", "SPTrack.UpdateDescriptor"},
    };
    for (const auto& [param, key] : int_overrides_) {
      declare_parameter<int>(param, -1);
    }
    declare_parameter<std::string>("superpoint_model_path", "");
    declare_parameter<std::string>("stats_csv", "");
    // Any other config keys, e.g. "SuperPoint.DetThreshold=0.005;SPTrack.SearchRadius=32"
    declare_parameter<std::string>("config_overrides", "");
  }

  void start()
  {
    if (config_path_.empty()) {
      throw std::runtime_error("rvio2_sp_mono_node requires parameter config_path");
    }

    const std::string effective_config = writeEffectiveConfig();
    system_ = std::make_unique<RVIO2SP::System>(effective_config);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 200, std::bind(&Rvio2SpMonoRos2::onImu, this, std::placeholders::_1));
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, 5, std::bind(&Rvio2SpMonoRos2::onImage, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "RVIO2-SP ROS 2 node consuming image=%s imu=%s config=%s",
                image_topic_.c_str(), imu_topic_.c_str(), effective_config.c_str());
  }

private:
  // Applies parameter overrides and resolves a relative SuperPoint model path
  // against the package share directory. Returns the config file to load.
  std::string writeEffectiveConfig()
  {
    std::map<std::string, std::string> overrides;
    for (const auto& [param, key] : int_overrides_) {
      const int value = get_parameter(param).as_int();
      if (value >= 0) {
        overrides[key] = std::to_string(value);
      }
    }

    std::string model_path = get_parameter("superpoint_model_path").as_string();
    if (model_path.empty()) {
      cv::FileStorage fs(config_path_, cv::FileStorage::READ);
      if (!fs.isOpened()) {
        throw std::runtime_error("rvio2_sp_mono_node: cannot open config " + config_path_);
      }
      if (!fs["SuperPoint.ModelPath"].empty()) {
        fs["SuperPoint.ModelPath"] >> model_path;
      }
    }
    if (!model_path.empty() && model_path.front() != '/') {
      model_path = ament_index_cpp::get_package_share_directory("mono_hydra_vio") + "/" + model_path;
    }
    if (!model_path.empty()) {
      overrides["SuperPoint.ModelPath"] = "\"" + model_path + "\"";
    }

    const std::string stats_csv = get_parameter("stats_csv").as_string();
    if (!stats_csv.empty()) {
      overrides["Stats.CsvPath"] = "\"" + stats_csv + "\"";
    }

    std::stringstream extra(get_parameter("config_overrides").as_string());
    std::string item;
    while (std::getline(extra, item, ';')) {
      const size_t eq = item.find('=');
      if (eq == std::string::npos) {
        continue;
      }
      const auto trim = [](std::string s) {
        s.erase(0, s.find_first_not_of(" \t"));
        s.erase(s.find_last_not_of(" \t") + 1);
        return s;
      };
      const std::string key = trim(item.substr(0, eq));
      if (!key.empty()) {
        overrides[key] = trim(item.substr(eq + 1));
      }
    }

    std::ifstream in(config_path_);
    if (!in) {
      throw std::runtime_error("rvio2_sp_mono_node: cannot open config " + config_path_);
    }

    std::ostringstream out;
    std::string line;
    while (std::getline(in, line)) {
      bool replaced = false;
      for (const auto& [key, value] : overrides) {
        const std::regex key_line("^\\s*" + std::regex_replace(key, std::regex("\\."), "\\.") + "\\s*:");
        if (std::regex_search(line, key_line)) {
          replaced = true;
          break;
        }
      }
      if (!replaced) {
        out << line << "\n";
      }
    }
    out << "\n# Overrides applied by rvio2_sp_mono_node\n";
    for (const auto& [key, value] : overrides) {
      out << key << ": " << value << "\n";
      RCLCPP_INFO(get_logger(), "config override %s: %s", key.c_str(), value.c_str());
    }

    const std::string path = "/tmp/rvio2_sp_config_" + std::to_string(getpid()) + ".yaml";
    std::ofstream file(path);
    file << out.str();
    return path;
  }

  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    } catch (const cv_bridge::Exception& exc) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge image conversion failed: %s", exc.what());
      return;
    }

    auto* data = new RVIO2SP::ImageData();
    data->Image = cv_ptr->image.clone();
    data->Timestamp = rclcpp::Time(cv_ptr->header.stamp).seconds();
    system_->PushImageData(data);
    system_->run();
  }

  void onImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    auto* data = new RVIO2SP::ImuData();
    data->AngularVel = Eigen::Vector3f(
      static_cast<float>(msg->angular_velocity.x),
      static_cast<float>(msg->angular_velocity.y),
      static_cast<float>(msg->angular_velocity.z));
    data->LinearAccel = Eigen::Vector3f(
      static_cast<float>(msg->linear_acceleration.x),
      static_cast<float>(msg->linear_acceleration.y),
      static_cast<float>(msg->linear_acceleration.z));
    data->Timestamp = rclcpp::Time(msg->header.stamp).seconds();
    static double last_time = -1.0;
    data->TimeInterval = last_time >= 0.0 ? data->Timestamp - last_time : 0.0;
    last_time = data->Timestamp;
    system_->PushImuData(data);
  }

  std::string image_topic_;
  std::string imu_topic_;
  std::string config_path_;
  std::map<std::string, std::string> int_overrides_;
  std::unique_ptr<RVIO2SP::System> system_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Rvio2SpMonoRos2>();
  ros::set_global_node(node);
  node->start();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
