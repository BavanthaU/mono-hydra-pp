#pragma once
#include <chrono>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/create_client.hpp>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/create_service.hpp>
#include <rclcpp/create_subscription.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_interfaces/node_clock_interface.hpp>
#include <rclcpp/node_interfaces/node_graph_interface.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/node_interfaces/node_services_interface.hpp>
#include <rclcpp/node_interfaces/node_timers_interface.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>

#include "ianvs/visibility_control.h"

namespace ianvs {

IANVS_PUBLIC std::string join_namespace(const std::string& ns, const std::string& topic);

class NodeHandle {
 public:
  template <typename T>
  using Publisher = typename rclcpp::Publisher<T>::SharedPtr;
  template <typename T>
  using Subscription = typename rclcpp::Subscription<T>::SharedPtr;
  template <typename T>
  using Service = typename rclcpp::Service<T>::SharedPtr;
  template <typename T>
  using Client = typename rclcpp::Client<T>::SharedPtr;
  using Timer = typename rclcpp::TimerBase::SharedPtr;
  using GroupPtr = rclcpp::CallbackGroup::SharedPtr;

  // TODO(nathan) this is really only one or two interfaces away from using all of them
  using NodeInterface =
      rclcpp::node_interfaces::NodeInterfaces<rclcpp::node_interfaces::NodeBaseInterface,
                                              rclcpp::node_interfaces::NodeClockInterface,
                                              rclcpp::node_interfaces::NodeLoggingInterface,
                                              rclcpp::node_interfaces::NodeGraphInterface,
                                              rclcpp::node_interfaces::NodeTimersInterface,
                                              rclcpp::node_interfaces::NodeTopicsInterface,
                                              rclcpp::node_interfaces::NodeServicesInterface,
                                              rclcpp::node_interfaces::NodeParametersInterface>;

  NodeHandle(NodeInterface node, const std::string& ns = "");
  NodeHandle& operator/=(const std::string& ns);
  operator NodeInterface() { return node_; }
  NodeInterface& node() { return node_; }
  const NodeInterface& node() const { return node_; }
  const std::string& ns() const { return ns_; }
  static NodeHandle this_node(const std::string& ns = "");

  template <typename InterfaceT>
  std::shared_ptr<InterfaceT> as();

  template <typename T>
  Publisher<T> create_publisher(const std::string& topic, const rclcpp::QoS& qos);

  template <typename T, typename CallbackT>
  Subscription<T> create_subscription(const std::string& topic,
                                      const rclcpp::QoS& qos,
                                      CallbackT&& callback,
                                      GroupPtr group = nullptr);

  template <typename T, typename CallbackT, typename Cls>
  Subscription<T> create_subscription(const std::string& topic,
                                      const rclcpp::QoS& qos,
                                      CallbackT&& callback,
                                      Cls* class_pointer,
                                      GroupPtr group = nullptr);

  template <typename T, typename CallbackT>
  Service<T> create_service(const std::string& service_name,
                            CallbackT&& callback,
                            const rclcpp::QoS& qos = rclcpp::ServicesQoS(),
                            GroupPtr group = nullptr);

  template <typename T, typename CallbackT, typename Cls>
  Service<T> create_service(const std::string& service_name,
                            CallbackT&& callback,
                            Cls* class_pointer,
                            const rclcpp::QoS& qos = rclcpp::ServicesQoS(),
                            GroupPtr group = nullptr);

  template <typename T>
  Client<T> create_client(const std::string& name,
                          const rclcpp::QoS& qos = rclcpp::ServicesQoS(),
                          GroupPtr = nullptr);

  template <typename CallbackT>
  Timer create_timer(std::chrono::milliseconds period_ms,
                     bool is_wall_time,
                     CallbackT&& callback,
                     GroupPtr group = nullptr);

  std::string resolve_name(const std::string& name, bool is_service) const;

  std::string node_name() const {
    using rclcpp::node_interfaces::NodeBaseInterface;
    return node_.get<NodeBaseInterface>()->get_fully_qualified_name();
  }

  rclcpp::Logger logger() const {
    return node_.get<rclcpp::node_interfaces::NodeLoggingInterface>()->get_logger();
  }

  rclcpp::Time now() const {
    return node_.get<rclcpp::node_interfaces::NodeClockInterface>()->get_clock()->now();
  }

  rclcpp::Clock::SharedPtr clock() {
    return node_.get<rclcpp::node_interfaces::NodeClockInterface>()->get_clock();
  }

 private:
  NodeInterface node_;
  std::string ns_;
};

template <typename InterfaceT>
std::shared_ptr<InterfaceT> NodeHandle::as() {
  return node_.get<InterfaceT>();
}

template <typename T>
typename NodeHandle::Publisher<T> NodeHandle::create_publisher(const std::string& topic,
                                                               const rclcpp::QoS& qos) {
  return rclcpp::create_publisher<T>(node_, join_namespace(ns_, topic), qos);
}

template <typename T, typename CallbackT>
typename NodeHandle::Subscription<T> NodeHandle::create_subscription(const std::string& topic,
                                                                     const rclcpp::QoS& qos,
                                                                     CallbackT&& callback,
                                                                     GroupPtr group) {
  rclcpp::SubscriptionOptions opts;
  opts.callback_group = group;
  return rclcpp::create_subscription<T>(node_, join_namespace(ns_, topic), qos, callback, opts);
}

template <typename T, typename CallbackT, typename Cls>
typename NodeHandle::Subscription<T> NodeHandle::create_subscription(const std::string& topic,
                                                                     const rclcpp::QoS& qos,
                                                                     CallbackT&& callback,
                                                                     Cls* class_pointer,
                                                                     GroupPtr group) {
  return create_subscription<T>(
      topic, qos, std::bind(callback, class_pointer, std::placeholders::_1));
}

template <typename T, typename CallbackT>
typename NodeHandle::Service<T> NodeHandle::create_service(const std::string& name,
                                                           CallbackT&& callback,
                                                           const rclcpp::QoS& qos,
                                                           GroupPtr group) {
  // NOTE(nathan) this is necesary because (unlike pub/sub) no one has updated
  // create_service to take a single node interface argument as of iron. Ideally we'd
  // replace this once the API updates. Note that the actual node implementation will
  // append a subnode namespace if it exists, but because subnodes are poorly designed
  // and don't get added to executors by default, we're not going to support this
  auto base = node_.get<rclcpp::node_interfaces::NodeBaseInterface>();
  auto services = node_.get<rclcpp::node_interfaces::NodeServicesInterface>();
  const auto new_name = join_namespace(ns_, name);
  return rclcpp::create_service<T>(base, services, new_name, callback, qos, group);
}

template <typename T, typename CallbackT, typename Cls>
typename NodeHandle::Service<T> NodeHandle::create_service(const std::string& name,
                                                           CallbackT&& callback,
                                                           Cls* class_pointer,
                                                           const rclcpp::QoS& qos,
                                                           GroupPtr group) {
  return create_service<T>(
      join_namespace(ns_, name),
      std::bind(callback, class_pointer, std::placeholders::_1, std::placeholders::_2),
      qos,
      group);
}

template <typename T>
typename NodeHandle::Client<T> NodeHandle::create_client(const std::string& name,
                                                         const rclcpp::QoS& qos,
                                                         GroupPtr group) {
  // NOTE(nathan) this is necesary because (unlike pub/sub) no one has updated
  // create_client to take a single node interface argument as of iron. Ideally we'd
  // replace this once the API updates. Note that the actual node implementation will
  // append a subnode namespace if it exists, but because subnodes are poorly designed
  // and don't get added to executors by default, we're not going to support this
  auto base = node_.get<rclcpp::node_interfaces::NodeBaseInterface>();
  auto graph = node_.get<rclcpp::node_interfaces::NodeGraphInterface>();
  auto services = node_.get<rclcpp::node_interfaces::NodeServicesInterface>();
  const auto new_name = join_namespace(ns_, name);
  return rclcpp::create_client<T>(base, graph, services, new_name, qos, group);
}

template <typename CallbackT>
NodeHandle::Timer NodeHandle::create_timer(std::chrono::milliseconds period_ms,
                                           bool is_wall_time,
                                           CallbackT&& callback,
                                           GroupPtr group) {
  auto base = node_.get<rclcpp::node_interfaces::NodeBaseInterface>();
  auto timer = node_.get<rclcpp::node_interfaces::NodeTimersInterface>();
  if (is_wall_time) {
    auto to_return = rclcpp::create_wall_timer(period_ms, callback, group, base.get(), timer.get());
    // force autostart
    to_return->reset();
    return to_return;
  }

  auto clock = node_.get<rclcpp::node_interfaces::NodeClockInterface>();
  auto to_return =
      rclcpp::create_timer(clock->get_clock(), period_ms, callback, group, base.get(), timer.get());
  to_return->reset();
  return to_return;
}

inline NodeHandle operator/(const NodeHandle& nh, const std::string& ns) {
  return NodeHandle(nh.node(), join_namespace(nh.ns(), ns));
}

template <typename T>
typename T::Response::SharedPtr call_service(rclcpp::Client<T>& client,
                                             std::shared_ptr<typename T::Request> req,
                                             size_t timeout_ms = 0,
                                             NodeHandle* const nh = nullptr) {
  using namespace std::chrono_literals;

  auto start = std::chrono::steady_clock::now();
  const std::chrono::milliseconds timeout(timeout_ms);
  while (!client.wait_for_service(10ms) && rclcpp::ok()) {
    const std::chrono::duration<double> diff = std::chrono::steady_clock::now() - start;
    if (timeout > std::chrono::milliseconds::zero() && diff > timeout) {
      return nullptr;
    }

    // TODO(nathan) logging

    if (nh) {
      auto base = nh->node().get<rclcpp::node_interfaces::NodeBaseInterface>();
      rclcpp::spin_some(base);
    }
  }

  bool valid = false;
  auto result = client.async_send_request(req);
  while (rclcpp::ok()) {
    const std::chrono::duration<double> diff = std::chrono::steady_clock::now() - start;
    if (timeout > std::chrono::milliseconds::zero() && diff > timeout) {
      break;
    }

    if (nh) {
      auto base = nh->node().get<rclcpp::node_interfaces::NodeBaseInterface>();
      rclcpp::spin_some(base);
    }

    if (result.wait_for(10ms) == std::future_status::ready) {
      valid = true;
      break;
    }
  }

  return valid ? result.get() : nullptr;
}

}  // namespace ianvs
