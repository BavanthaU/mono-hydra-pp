#include "ianvs/image_publisher.h"

#include <rclcpp/create_publisher.hpp>

namespace ianvs {

struct ImagePublisher::Impl {
  Impl(NodeInterface node, const std::string& topic, const rclcpp::QoS& qos) {
    pub = rclcpp::create_publisher<sensor_msgs::msg::Image>(node, topic, qos);
  }

  void publish(const sensor_msgs::msg::Image& msg) const { pub->publish(msg); }

  void publish(sensor_msgs::msg::Image::UniquePtr&& msg) const { pub->publish(std::move(msg)); }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;
};

ImagePublisher::ImagePublisher() = default;

ImagePublisher::ImagePublisher(NodeInterface node, const std::string& topic, const rclcpp::QoS& qos)
    : impl_(std::make_unique<Impl>(node, topic, qos)) {}

ImagePublisher::~ImagePublisher() = default;

ImagePublisher::ImagePublisher(ImagePublisher&& other) { impl_ = std::move(other.impl_); }

ImagePublisher& ImagePublisher::operator=(ImagePublisher&& other) {
  impl_ = std::move(other.impl_);
  return *this;
}

void ImagePublisher::publish(const sensor_msgs::msg::Image& msg) const {
  if (impl_) {
    impl_->publish(msg);
  } else {
    throw std::runtime_error("unitialized publisher!");
  }
}

void ImagePublisher::publish(const sensor_msgs::msg::Image::ConstSharedPtr& msg) const {
  publish(*msg);
}

void ImagePublisher::publish(sensor_msgs::msg::Image::UniquePtr&& msg) const {
  if (impl_) {
    impl_->publish(std::move(msg));
  } else {
    throw std::runtime_error("unitialized publisher!");
  }
}

}  // namespace ianvs
