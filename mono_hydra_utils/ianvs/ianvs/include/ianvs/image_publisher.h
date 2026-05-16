#pragma once
#include <memory>

#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace ianvs {

class ImagePublisher {
 public:
  using NodeInterface =
      rclcpp::node_interfaces::NodeInterfaces<rclcpp::node_interfaces::NodeParametersInterface,
                                              rclcpp::node_interfaces::NodeTopicsInterface>;

  ImagePublisher();
  ImagePublisher(NodeInterface node,
                 const std::string& topic,
                 const rclcpp::QoS& qos = rclcpp::SensorDataQoS());
  ~ImagePublisher();
  ImagePublisher(ImagePublisher&& other);
  ImagePublisher& operator=(ImagePublisher&& other);

  void publish(const sensor_msgs::msg::Image& msg) const;

  void publish(const sensor_msgs::msg::Image::ConstSharedPtr& msg) const;

  void publish(sensor_msgs::msg::Image::UniquePtr&& msg) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ianvs
