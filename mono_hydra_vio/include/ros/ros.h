#pragma once

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace geometry_msgs
{
using Point = msg::Point;
using Vector3 = msg::Vector3;
}

namespace std_msgs
{
using ColorRGBA = msg::ColorRGBA;
}

namespace ros
{
inline std::shared_ptr<rclcpp::Node>& global_node()
{
  static std::shared_ptr<rclcpp::Node> node;
  return node;
}

inline void set_global_node(const std::shared_ptr<rclcpp::Node>& node)
{
  global_node() = node;
}

class Time
{
public:
  Time() = default;
  explicit Time(double seconds)
  {
    sec_ = static_cast<int32_t>(seconds);
    nanosec_ = static_cast<uint32_t>((seconds - static_cast<double>(sec_)) * 1e9);
  }
  double toSec() const { return static_cast<double>(sec_) + static_cast<double>(nanosec_) * 1e-9; }
  operator builtin_interfaces::msg::Time() const
  {
    builtin_interfaces::msg::Time out;
    out.sec = sec_;
    out.nanosec = nanosec_;
    return out;
  }

private:
  int32_t sec_{0};
  uint32_t nanosec_{0};
};

class Duration
{
public:
  explicit Duration(double seconds)
  {
    sec_ = static_cast<int32_t>(seconds);
    nanosec_ = static_cast<uint32_t>((seconds - static_cast<double>(sec_)) * 1e9);
  }
  operator builtin_interfaces::msg::Duration() const
  {
    builtin_interfaces::msg::Duration out;
    out.sec = sec_;
    out.nanosec = nanosec_;
    return out;
  }

private:
  int32_t sec_{0};
  uint32_t nanosec_{0};
};

class Publisher
{
  struct HolderBase
  {
    virtual ~HolderBase() = default;
  };

  template <typename MsgT>
  struct Holder : HolderBase
  {
    explicit Holder(const std::shared_ptr<rclcpp::Publisher<MsgT>>& pub_in) : pub(pub_in) {}
    std::shared_ptr<rclcpp::Publisher<MsgT>> pub;
  };

public:
  Publisher() = default;
  template <typename MsgT>
  explicit Publisher(const std::shared_ptr<rclcpp::Publisher<MsgT>>& pub) : holder_(std::make_shared<Holder<MsgT>>(pub)) {}

  template <typename MsgT>
  void publish(const MsgT& msg)
  {
    auto typed = std::dynamic_pointer_cast<Holder<MsgT>>(holder_);
    if (typed && typed->pub) {
      typed->pub->publish(msg);
    }
  }

  template <typename MsgT>
  void publish(const std::shared_ptr<MsgT>& msg)
  {
    if (msg) {
      publish<MsgT>(*msg);
    }
  }

private:
  std::shared_ptr<HolderBase> holder_;
};

class NodeHandle
{
public:
  explicit NodeHandle(const std::string& ns = "") : ns_(ns) {}

  template <typename MsgT>
  Publisher advertise(const std::string& topic, int queue_size)
  {
    auto node = global_node();
    if (!node) {
      return Publisher();
    }
    return Publisher(node->create_publisher<MsgT>(topic, static_cast<size_t>(queue_size)));
  }

  template <typename T>
  void param(const std::string& name, T& variable, const T& default_value)
  {
    auto node = global_node();
    if (!node) {
      variable = default_value;
      return;
    }
    const std::string key = ns_ == "~" ? name : name;
    if (!node->has_parameter(key)) {
      node->declare_parameter<T>(key, default_value);
    }
    variable = node->get_parameter(key).get_value<T>();
  }

private:
  std::string ns_;
};
}  // namespace ros

#define ROS_INFO(...) RCLCPP_INFO(::ros::global_node()->get_logger(), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(::ros::global_node()->get_logger(), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(::ros::global_node()->get_logger(), __VA_ARGS__)
#define ROS_DEBUG(...) RCLCPP_DEBUG(::ros::global_node()->get_logger(), __VA_ARGS__)
#define ROS_INFO_THROTTLE(period, ...) \
  RCLCPP_INFO_THROTTLE(::ros::global_node()->get_logger(), *::ros::global_node()->get_clock(), static_cast<int>((period) * 1000.0), __VA_ARGS__)
