#include <algorithm>
#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <kimera_pgmo_msgs/msg/mesh.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace {

using Mesh = kimera_pgmo_msgs::msg::Mesh;
using Marker = visualization_msgs::msg::Marker;

std_msgs::msg::ColorRGBA makeColor(const std::vector<double>& values,
                                   double alpha) {
  std_msgs::msg::ColorRGBA color;
  color.r = static_cast<float>(values.size() > 0 ? values[0] : 0.62);
  color.g = static_cast<float>(values.size() > 1 ? values[1] : 0.64);
  color.b = static_cast<float>(values.size() > 2 ? values[2] : 0.68);
  color.a = static_cast<float>(alpha);
  return color;
}

}  // namespace

class MeshMarkerNode : public rclcpp::Node {
 public:
  MeshMarkerNode() : Node("mono_hydra_dsg_mesh_marker") {
    input_topic_ =
        declare_parameter<std::string>("input_mesh_topic", "/hydra/backend/dsg_mesh");
    output_topic_ = declare_parameter<std::string>(
        "output_marker_topic", "/hydra_dsg_visualizer/dsg_mesh_marker");
    fallback_frame_id_ = declare_parameter<std::string>("fallback_frame_id", "map");
    mesh_alpha_ = declare_parameter<double>("mesh_alpha", 0.92);
    default_color_ =
        makeColor(declare_parameter<std::vector<double>>(
                      "default_color", std::vector<double>{0.72, 0.74, 0.78, 0.92}),
                  mesh_alpha_);
    max_triangles_ = declare_parameter<int>("max_triangles", 0);
    warn_conversion_s_ = declare_parameter<double>("warn_conversion_s", 0.5);

    auto input_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    publisher_ = create_publisher<Marker>(output_topic_, output_qos);
    subscription_ =
        create_subscription<Mesh>(input_topic_, input_qos, [this](Mesh::ConstSharedPtr msg) {
          onMesh(msg);
        });

    RCLCPP_INFO(get_logger(),
                "Rendering Kimera-PGMO mesh '%s' as RViz Marker '%s'",
                input_topic_.c_str(),
                output_topic_.c_str());
  }

 private:
  void publishDelete(const Mesh& msg) {
    Marker marker;
    marker.header = msg.header;
    if (marker.header.frame_id.empty()) {
      marker.header.frame_id = fallback_frame_id_;
    }
    marker.ns = "dsg_mesh";
    marker.id = 0;
    marker.action = Marker::DELETE;
    publisher_->publish(marker);
  }

  std_msgs::msg::ColorRGBA vertexColor(const Mesh::_vertices_type::value_type& vertex) const {
    if (!vertex.has_color) {
      return default_color_;
    }

    auto color = vertex.color;
    color.a = static_cast<float>(mesh_alpha_);
    return color;
  }

  void onMesh(const Mesh::ConstSharedPtr& msg) {
    const auto start = std::chrono::steady_clock::now();
    if (msg->vertices.empty() || msg->triangles.empty()) {
      publishDelete(*msg);
      return;
    }

    Marker marker;
    marker.header = msg->header;
    if (marker.header.frame_id.empty()) {
      marker.header.frame_id = fallback_frame_id_;
    }
    marker.ns = "dsg_mesh";
    marker.id = 0;
    marker.type = Marker::TRIANGLE_LIST;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color = default_color_;

    const size_t triangle_count =
        max_triangles_ > 0
            ? std::min(msg->triangles.size(), static_cast<size_t>(max_triangles_))
            : msg->triangles.size();
    marker.points.reserve(triangle_count * 3);
    marker.colors.reserve(triangle_count * 3);

    for (size_t i = 0; i < triangle_count; ++i) {
      const auto& triangle = msg->triangles[i];
      for (const auto index : triangle.vertex_indices) {
        if (index >= msg->vertices.size()) {
          continue;
        }
        const auto& vertex = msg->vertices[index];
        marker.points.push_back(vertex.pos);
        marker.colors.push_back(vertexColor(vertex));
      }
    }

    if (marker.points.empty()) {
      publishDelete(*msg);
      return;
    }

    publisher_->publish(marker);

    const auto elapsed = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - start)
                             .count();
    if (elapsed > warn_conversion_s_) {
      RCLCPP_WARN_THROTTLE(get_logger(),
                           *get_clock(),
                           5000,
                           "Mesh marker conversion took %.2fs for %zu triangles",
                           elapsed,
                           triangle_count);
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string fallback_frame_id_;
  double mesh_alpha_;
  std_msgs::msg::ColorRGBA default_color_;
  int max_triangles_;
  double warn_conversion_s_;
  rclcpp::Publisher<Marker>::SharedPtr publisher_;
  rclcpp::Subscription<Mesh>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MeshMarkerNode>());
  rclcpp::shutdown();
  return 0;
}
