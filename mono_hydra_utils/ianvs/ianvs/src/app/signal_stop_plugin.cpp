#include <std_srvs/srv/empty.hpp>

#include "ianvs/app/rosbag_play_plugins.h"
#include "ianvs/node_handle.h"

namespace ianvs {

class SignalStopPlugin : public RosbagPlayPlugin {
 public:
  SignalStopPlugin() = default;

  void init(std::shared_ptr<rclcpp::Node> node) override;
  void add_options(CLI::App& app) override;
  void on_start(rosbag2_cpp::Reader&,
                rosbag2_transport::PlayOptions&,
                const rclcpp::Logger*) override {}
  void on_stop() override;

 private:
  std::vector<std::string> srvs_;
  std::shared_ptr<rclcpp::Node> node_;
  std::vector<rclcpp::Client<std_srvs::srv::Empty>::SharedPtr> clients_;
};

void SignalStopPlugin::init(std::shared_ptr<rclcpp::Node> node) {
  node_ = node;
  if (!node_) {
    return;
  }

  for (const auto& srv : srvs_) {
    clients_.push_back(node_->create_client<std_srvs::srv::Empty>(srv));
  }
}

void SignalStopPlugin::add_options(CLI::App& app) {
  app.add_option("--signal-services", srvs_)->description("services to call when rosbag stops");
}

void SignalStopPlugin::on_stop() {
  if (!rclcpp::ok() || !node_) {
    return;
  }

  for (const auto& client : clients_) {
    auto req = std::make_shared<std_srvs::srv::Empty::Request>();
    const auto rep = ianvs::call_service(*client, req, 50);
    if (!rep) {
      RCLCPP_ERROR_STREAM(
          node_->get_logger(),
          "Service call for client '" << std::string(client->get_service_name()) << "' failed!");
    }
  }
}

}  // namespace ianvs

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ianvs::SignalStopPlugin, ianvs::RosbagPlayPlugin)
