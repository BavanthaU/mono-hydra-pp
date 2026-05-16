#include "ianvs/node_init.h"

namespace ianvs {

std::unique_ptr<CurrentNode> CurrentNode::s_instance_ = nullptr;

void CurrentNode::init(std::shared_ptr<rclcpp::Node> node) {
  auto& curr = instance();
  if (curr.node_) {
    throw std::runtime_error("Current node already set!");
  }

  curr.node_ = node;
}

rclcpp::Node* CurrentNode::get() { return instance().node_.get(); }

void CurrentNode::clear() { instance().node_.reset(); }

CurrentNode& CurrentNode::instance() {
  if (!s_instance_) {
    s_instance_.reset(new CurrentNode());
  }

  return *s_instance_;
}

CurrentNode::CurrentNode() {}

NodeGuard::~NodeGuard() { CurrentNode::clear(); }

NodeGuard init_node(int& argc, char** argv, const std::string& node_name) {
  rclcpp::init(argc, argv);
  CurrentNode::init(std::make_shared<rclcpp::Node>(node_name));
  return {};
}

}  // namespace ianvs
