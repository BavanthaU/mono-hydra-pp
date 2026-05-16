#pragma once
#include <message_filters/simple_filter.h>

#include <memory>

#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/node_interfaces/node_timers_interface.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace ianvs {

class ImageSubscription : public message_filters::SimpleFilter<sensor_msgs::msg::Image> {
 public:
  using NodeInterface =
      rclcpp::node_interfaces::NodeInterfaces<rclcpp::node_interfaces::NodeParametersInterface,
                                              rclcpp::node_interfaces::NodeTimersInterface,
                                              rclcpp::node_interfaces::NodeTopicsInterface>;

  using Callback = std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr&)>;

  ImageSubscription(NodeInterface node);
  ImageSubscription(NodeInterface node,
                    const std::string& topic,
                    const rclcpp::QoS& qos = rclcpp::SensorDataQoS(),
                    const Callback& callback = Callback());

  template <typename MethodT, typename ObjT>
  ImageSubscription(NodeInterface node,
                    const std::string& topic,
                    const MethodT& method,
                    ObjT* obj,
                    const rclcpp::QoS& qos = rclcpp::SensorDataQoS())
      : ImageSubscription(node, topic, qos, std::bind(method, obj, std::placeholders::_1)) {}

  ~ImageSubscription();
  ImageSubscription(const ImageSubscription& other) = delete;
  ImageSubscription& operator=(const ImageSubscription& other) = delete;
  ImageSubscription(ImageSubscription&& other) = delete;
  ImageSubscription& operator=(ImageSubscription&& other) = delete;

  void subscribe(const std::string& topic, const rclcpp::QoS& qos = rclcpp::SensorDataQoS());
  void unsubscribe();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ianvs
