#include <memory>
#include <string>

#include <Eigen/Core>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "ros/ros.h"
#include "rvio2/System.h"

class Rvio2MonoRos2 : public rclcpp::Node
{
public:
  Rvio2MonoRos2() : Node("rvio2_mono_node")
  {
    image_topic_ = declare_parameter<std::string>("image_topic", "/rvio2_bridge/cam0/image_raw");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/camera/imu");
    config_path_ = declare_parameter<std::string>("config_path", "");
  }

  void start()
  {
    if (config_path_.empty()) {
      throw std::runtime_error("rvio2_mono_node requires parameter config_path");
    }

    system_ = std::make_unique<RVIO2::System>(config_path_);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 200, std::bind(&Rvio2MonoRos2::onImu, this, std::placeholders::_1));
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, 5, std::bind(&Rvio2MonoRos2::onImage, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "RVIO2 ROS 2 node consuming image=%s imu=%s config=%s",
                image_topic_.c_str(), imu_topic_.c_str(), config_path_.c_str());
  }

private:
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    } catch (const cv_bridge::Exception& exc) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge image conversion failed: %s", exc.what());
      return;
    }

    auto* data = new RVIO2::ImageData();
    data->Image = cv_ptr->image.clone();
    data->Timestamp = rclcpp::Time(cv_ptr->header.stamp).seconds();
    system_->PushImageData(data);
    system_->run();
  }

  void onImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    auto* data = new RVIO2::ImuData();
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
  std::unique_ptr<RVIO2::System> system_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Rvio2MonoRos2>();
  ros::set_global_node(node);
  node->start();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
