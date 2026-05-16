/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#pragma once
#include <map>
#include <string>

#include "ianvs/node_handle.h"

namespace ianvs {

template <typename Derived>
struct publisher_type_trait;

// NOTE(nathan) CRTP or something similar seems necessary here becuase the various
// publisher types need something common (the Derived class) to hold construction
// information
template <typename Derived>
class LazyPublisherGroup {
 public:
  virtual ~LazyPublisherGroup() = default;

  template <typename Callback>
  void publish(const std::string& topic, const Callback& callback) const {
    const auto derived = static_cast<const Derived*>(this);
    auto iter = pubs_.find(topic);
    if (iter == pubs_.end()) {
      iter = pubs_.emplace(topic, derived->make_publisher(topic)).first;
    }

    if (!derived->should_publish(iter->second)) {
      return;
    }

    derived->publish_msg(iter->second, callback());
  }

 private:
  friend Derived;
  LazyPublisherGroup() = default;

  using PublisherT = typename publisher_type_trait<Derived>::value;
  mutable std::map<std::string, PublisherT> pubs_;
};

template <typename T>
struct RosPublisherGroup;

template <typename T>
struct publisher_type_trait<RosPublisherGroup<T>> {
  using value = typename rclcpp::Publisher<T>::SharedPtr;
};

template <typename T>
struct RosPublisherGroup : LazyPublisherGroup<RosPublisherGroup<T>> {
 public:
  using Base = LazyPublisherGroup<RosPublisherGroup<T>>;
  using Pub = typename rclcpp::Publisher<T>::SharedPtr;

  explicit RosPublisherGroup(NodeHandle nh, const rclcpp::QoS& qos = rclcpp::QoS(1))
      : nh_(nh), qos(qos) {}

  Pub make_publisher(const std::string& topic) const { return nh_.create_publisher<T>(topic, qos); }

  bool should_publish(const Pub& pub) const { return pub->get_subscription_count() > 0; }

  void publish_msg(const Pub& pub, typename T::UniquePtr&& msg) const {
    pub->publish(std::move(msg));
  }

  void publish_msg(const Pub& pub, const T& msg) const { pub->publish(msg); }

  void publish_msg(const Pub& pub, const typename T::ConstSharedPtr& msg) const {
    pub->publish(*msg);
  }

  const rclcpp::QoS qos;

 private:
  mutable NodeHandle nh_;
};

}  // namespace ianvs
