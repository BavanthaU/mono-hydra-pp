#include "ianvs/image_subscription.h"

namespace ianvs {

using message_filters::Connection;
using sensor_msgs::msg::Image;

struct ImageSubscription::Impl {
  Impl(NodeInterface node, const ImageSubscription::Callback& callback)
      : node(node), callback(callback) {}

  void subscribe(const std::string& topic, const rclcpp::QoS& qos) {
    sub = rclcpp::create_subscription<Image>(node, topic, qos, callback);
  }

  void unsubscribe() { sub.reset(); }

  NodeInterface node;
  ImageSubscription::Callback callback;
  rclcpp::Subscription<Image>::SharedPtr sub;
};

ImageSubscription::ImageSubscription(NodeInterface node)
    : impl_(std::make_unique<Impl>(
          node, [this](const Image::ConstSharedPtr& msg) { signalMessage(msg); })) {}

ImageSubscription::ImageSubscription(NodeInterface node,
                                     const std::string& topic,
                                     const rclcpp::QoS& qos,
                                     const Callback& callback)
    : impl_(std::make_unique<Impl>(
          node, [this](const Image::ConstSharedPtr& msg) { signalMessage(msg); })) {
  impl_->subscribe(topic, qos);
  if (callback) {
    registerCallback(callback);
  }
}

ImageSubscription::~ImageSubscription() = default;

void ImageSubscription::subscribe(const std::string& topic, const rclcpp::QoS& qos) {
  impl_->subscribe(topic, qos);
}

void ImageSubscription::unsubscribe() { impl_->unsubscribe(); }

}  // namespace ianvs
