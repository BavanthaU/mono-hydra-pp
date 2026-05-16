#pragma once
#include <rclcpp/executors.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>

#include "ianvs/node_handle.h"

namespace ianvs {

template <typename MsgT>
struct MessageWaitFunctor {
  using PtrT = typename MsgT::ConstSharedPtr;
  void callback(const PtrT& msg) { msg_ = msg; }

  PtrT wait(NodeHandle nh, bool spin_required, size_t timeout_ms = 0) {
    auto base = nh.node().get<rclcpp::node_interfaces::NodeBaseInterface>();
    const auto start = std::chrono::steady_clock::now();
    const std::chrono::milliseconds timeout(timeout_ms);

    rclcpp::WallRate r(10.0);
    while (rclcpp::ok() && !msg_) {
      if (spin_required) {
        rclcpp::spin_some(base);
      }

      if (!msg_) {
        r.sleep();
      }

      const std::chrono::duration<double> diff = std::chrono::steady_clock::now() - start;
      if (timeout > std::chrono::milliseconds::zero() && diff > timeout) {
        break;
      }
    }

    return msg_;
  }

 private:
  PtrT msg_;
};

template <typename MsgT>
std::optional<MsgT> getSingleMessage(NodeHandle nh,
                                     const std::string& topic,
                                     bool spin_required,
                                     const rclcpp::QoS& qos = rclcpp::QoS(1),
                                     size_t timeout_ms = 0) {
  MessageWaitFunctor<MsgT> functor;
  [[maybe_unused]] const auto sub =
      nh.create_subscription<MsgT>(topic, qos, &MessageWaitFunctor<MsgT>::callback, &functor);
  const auto msg = functor.wait(nh, spin_required, timeout_ms);
  if (!msg) {
    return std::nullopt;
  }

  return *msg;
}

}  // namespace ianvs
