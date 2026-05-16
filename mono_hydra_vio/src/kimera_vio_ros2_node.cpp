#include <gflags/gflags.h>
#include <glog/logging.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/imgproc.hpp>
#include <pose_graph_tools_msgs/msg/pose_graph.hpp>
#include <pose_graph_tools_msgs/msg/pose_graph_edge.hpp>
#include <pose_graph_tools_msgs/msg/pose_graph_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/slam/BetweenFactor.h>

#include "kimera-vio/backend/VioBackend-definitions.h"
#include "kimera-vio/frontend/DepthFrame.h"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/loopclosure/LcdOutputPacket.h"
#include "kimera-vio/pipeline/MonoImuPipeline.h"
#include "kimera-vio/pipeline/RgbdImuPipeline.h"
#include "kimera-vio/pipeline/StereoImuPipeline.h"
#include "kimera-vio/pipeline/Pipeline-definitions.h"
#include "kimera-vio/utils/Macros.h"
#include "kimera-vio/visualizer/Display.h"
#include "kimera-vio/visualizer/Visualizer3D.h"

DECLARE_bool(log_output);
DECLARE_bool(use_external_odometry);
DECLARE_bool(use_lcd);
DECLARE_bool(visualize);
DECLARE_bool(lcd_no_optimize);
DECLARE_bool(lcd_no_detection);
DECLARE_bool(lcd_disable_stereo_match_depth_check);
DECLARE_bool(no_incremental_pose);
DECLARE_bool(do_coarse_imu_camera_temporal_sync);
DECLARE_bool(do_fine_imu_camera_temporal_sync);
DECLARE_int32(viz_type);

namespace mono_hydra_vio_ros2 {

namespace pgt = pose_graph_tools_msgs::msg;
using ImageMsg = sensor_msgs::msg::Image;
using ImagePtr = ImageMsg::ConstSharedPtr;
using ImuMsg = sensor_msgs::msg::Imu;
using OdometryMsg = nav_msgs::msg::Odometry;
using PoseBetween = gtsam::BetweenFactor<gtsam::Pose3>;
using RgbdSyncPolicy =
    message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg>;
using StereoSyncPolicy =
    message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg>;

builtin_interfaces::msg::Time stampFromNs(const VIO::Timestamp& stamp_ns) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
  stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
  return stamp;
}

VIO::Timestamp stampToNs(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<VIO::Timestamp>(stamp.sec) * 1000000000LL +
         static_cast<VIO::Timestamp>(stamp.nanosec);
}

void poseToTransform(const gtsam::Pose3& pose,
                     geometry_msgs::msg::Transform* transform) {
  CHECK_NOTNULL(transform);
  transform->translation.x = pose.x();
  transform->translation.y = pose.y();
  transform->translation.z = pose.z();
  const gtsam::Quaternion quat = pose.rotation().toQuaternion();
  transform->rotation.x = quat.x();
  transform->rotation.y = quat.y();
  transform->rotation.z = quat.z();
  transform->rotation.w = quat.w();
}

void poseToMsg(const gtsam::Pose3& pose, geometry_msgs::msg::Pose* msg) {
  CHECK_NOTNULL(msg);
  msg->position.x = pose.x();
  msg->position.y = pose.y();
  msg->position.z = pose.z();
  const gtsam::Quaternion quat = pose.rotation().toQuaternion();
  msg->orientation.x = quat.x();
  msg->orientation.y = quat.y();
  msg->orientation.z = quat.z();
  msg->orientation.w = quat.w();
}

gtsam::Pose3 odometryToPose(const OdometryMsg& odom) {
  const auto& q = odom.pose.pose.orientation;
  const auto& p = odom.pose.pose.position;
  return gtsam::Pose3(gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z),
                      gtsam::Point3(p.x, p.y, p.z));
}

class KimeraLcdPublisher {
 public:
  explicit KimeraLcdPublisher(rclcpp::Node& node)
      : node_(node),
        tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(node)) {
    odom_frame_id_ = getOrDeclare<std::string>("odom_frame_id", "odom");
    base_link_frame_id_ =
        getOrDeclare<std::string>("base_link_frame_id", "base_link_kimera");
    map_frame_id_ = getOrDeclare<std::string>("map_frame_id", "map");
    robot_id_ = static_cast<uint16_t>(getOrDeclare<int>("robot_id", 0));
    publish_lcd_tf_ = getOrDeclare<bool>("publish_lcd_tf", true);

    trajectory_pub_ =
        node_.create_publisher<nav_msgs::msg::Path>("optimized_trajectory", 1);
    posegraph_pub_ = node_.create_publisher<pgt::PoseGraph>("pose_graph", 1);
    posegraph_incremental_pub_ =
        node_.create_publisher<pgt::PoseGraph>("pose_graph_incremental", 1000);
    odometry_pub_ =
        node_.create_publisher<nav_msgs::msg::Odometry>("optimized_odometry", 1);
  }

  void publishLcdOutput(const VIO::LcdOutput::ConstPtr& lcd_output) {
    CHECK(lcd_output);
    publishTf(lcd_output);
    publishOptimizedTrajectory(lcd_output);
    publishPoseGraph(lcd_output);
  }

 private:
  template <typename T>
  T getOrDeclare(const std::string& name, const T& default_value) {
    if (!node_.has_parameter(name)) {
      node_.declare_parameter<T>(name, default_value);
    }
    return node_.get_parameter(name).get_value<T>();
  }

  void publishTf(const VIO::LcdOutput::ConstPtr& lcd_output) {
    if (!publish_lcd_tf_) {
      return;
    }
    geometry_msgs::msg::TransformStamped map_tf;
    map_tf.header.stamp = stampFromNs(lcd_output->timestamp_);
    map_tf.header.frame_id = map_frame_id_;
    map_tf.child_frame_id = odom_frame_id_;
    poseToTransform(lcd_output->Map_Pose_Odom_, &map_tf.transform);
    tf_broadcaster_->sendTransform(map_tf);
  }

  void publishOptimizedTrajectory(
      const VIO::LcdOutput::ConstPtr& lcd_output) {
    const VIO::Timestamp& ts = lcd_output->timestamp_;
    const auto& times = lcd_output->timestamp_map_;
    const auto& trajectory = lcd_output->states_;

    nav_msgs::msg::Path path;
    path.header.stamp = stampFromNs(ts);
    path.header.frame_id = map_frame_id_;
    path.poses.reserve(trajectory.size());

    gtsam::KeyVector keys = trajectory.keys();
    for (const auto& key : keys) {
      const auto frame_id = static_cast<VIO::FrameId>(gtsam::Symbol(key).index());
      if (!times.count(frame_id)) {
        continue;
      }
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = stampFromNs(times.at(frame_id));
      pose_msg.header.frame_id = map_frame_id_;
      poseToMsg(trajectory.at<gtsam::Pose3>(key), &pose_msg.pose);
      path.poses.push_back(pose_msg);
    }
    trajectory_pub_->publish(path);

    if (!keys.empty()) {
      nav_msgs::msg::Odometry odom_msg;
      odom_msg.header.stamp = stampFromNs(ts);
      odom_msg.header.frame_id = map_frame_id_;
      odom_msg.child_frame_id = base_link_frame_id_;
      poseToMsg(trajectory.at<gtsam::Pose3>(keys.back()),
                &odom_msg.pose.pose);
      odometry_pub_->publish(odom_msg);
    }
  }

  void updateRejectedEdges() {
    for (pgt::PoseGraphEdge& loop_edge : loop_closure_edges_) {
      bool is_inlier = false;
      for (const pgt::PoseGraphEdge& inlier_edge : inlier_edges_) {
        if (loop_edge.key_from == inlier_edge.key_from &&
            loop_edge.key_to == inlier_edge.key_to) {
          is_inlier = true;
          break;
        }
      }
      if (!is_inlier) {
        loop_edge.type = pgt::PoseGraphEdge::REJECTED_LOOPCLOSE;
      }
    }

    for (const pgt::PoseGraphEdge& inlier_edge : inlier_edges_) {
      bool stored = false;
      for (const pgt::PoseGraphEdge& loop_edge : loop_closure_edges_) {
        if (inlier_edge.key_from == loop_edge.key_from &&
            inlier_edge.key_to == loop_edge.key_to) {
          stored = true;
          break;
        }
      }
      if (!stored) {
        loop_closure_edges_.push_back(inlier_edge);
      }
    }
  }

  void updateNodesAndEdges(const VIO::FrameIDTimestampMap& times,
                           const gtsam::NonlinearFactorGraph& nfg,
                           const gtsam::Values& values) {
    inlier_edges_.clear();
    odometry_edges_.clear();

    for (size_t i = 0; i < nfg.size(); ++i) {
      const auto factor = dynamic_cast<const PoseBetween*>(nfg[i].get());
      if (!factor) {
        continue;
      }
      pgt::PoseGraphEdge edge;
      edge.header.frame_id = map_frame_id_;
      edge.key_from = factor->front();
      edge.key_to = factor->back();
      edge.robot_from = robot_id_;
      edge.robot_to = robot_id_;
      edge.type = edge.key_to == edge.key_from + 1
                      ? pgt::PoseGraphEdge::ODOM
                      : pgt::PoseGraphEdge::LOOPCLOSE;
      poseToMsg(factor->measured(), &edge.pose);
      edge.covariance.fill(0.0);
      if (edge.type == pgt::PoseGraphEdge::ODOM) {
        odometry_edges_.push_back(edge);
      } else {
        inlier_edges_.push_back(edge);
      }
    }

    updateRejectedEdges();
    pose_graph_nodes_.clear();
    gtsam::KeyVector keys = values.keys();
    for (const auto& key : keys) {
      const auto frame_id = static_cast<VIO::FrameId>(gtsam::Symbol(key).index());
      if (!times.count(frame_id)) {
        continue;
      }
      pgt::PoseGraphNode node;
      node.key = key;
      node.robot_id = robot_id_;
      node.header.stamp = stampFromNs(times.at(frame_id));
      node.header.frame_id = map_frame_id_;
      poseToMsg(values.at<gtsam::Pose3>(key), &node.pose);
      pose_graph_nodes_.push_back(node);
    }
  }

  pgt::PoseGraph poseGraphMsg() const {
    pgt::PoseGraph graph;
    graph.edges = odometry_edges_;
    graph.edges.insert(graph.edges.end(),
                       loop_closure_edges_.begin(),
                       loop_closure_edges_.end());
    graph.nodes = pose_graph_nodes_;
    return graph;
  }

  void publishPoseGraph(const VIO::LcdOutput::ConstPtr& lcd_output) {
    updateNodesAndEdges(
        lcd_output->timestamp_map_, lcd_output->nfg_, lcd_output->states_);

    pgt::PoseGraph graph = poseGraphMsg();
    graph.header.stamp = stampFromNs(lcd_output->timestamp_);
    graph.header.frame_id = map_frame_id_;
    posegraph_pub_->publish(graph);

    if (odometry_edges_.empty() || pose_graph_nodes_.size() < 2) {
      return;
    }

    pgt::PoseGraph incremental_graph;
    pgt::PoseGraphEdge last_odom_edge = odometry_edges_.back();
    last_odom_edge.header.stamp = stampFromNs(lcd_output->timestamp_);
    last_odom_edge.header.frame_id = map_frame_id_;
    last_odom_edge.type = pgt::PoseGraphEdge::ODOM;
    last_odom_edge.robot_from = robot_id_;
    last_odom_edge.robot_to = robot_id_;
    incremental_graph.edges.push_back(last_odom_edge);
    incremental_graph.nodes.push_back(
        pose_graph_nodes_.at(pose_graph_nodes_.size() - 2));
    incremental_graph.nodes.push_back(pose_graph_nodes_.back());

    if (lcd_output->is_loop_closure_) {
      pgt::PoseGraphEdge lc_edge;
      lc_edge.header.stamp = stampFromNs(lcd_output->timestamp_);
      lc_edge.header.frame_id = map_frame_id_;
      lc_edge.key_from = lcd_output->id_match_;
      lc_edge.key_to = lcd_output->id_recent_;
      lc_edge.robot_from = robot_id_;
      lc_edge.robot_to = robot_id_;
      lc_edge.type = pgt::PoseGraphEdge::LOOPCLOSE;
      lc_edge.covariance.fill(0.0);
      poseToMsg(lcd_output->relative_pose_, &lc_edge.pose);
      incremental_graph.edges.push_back(lc_edge);
      loop_closure_edges_.push_back(lc_edge);
      incremental_graph.edges.push_back(lc_edge);
    }

    incremental_graph.header.stamp = stampFromNs(lcd_output->timestamp_);
    incremental_graph.header.frame_id = map_frame_id_;
    posegraph_incremental_pub_->publish(incremental_graph);
  }

  rclcpp::Node& node_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<pgt::PoseGraph>::SharedPtr posegraph_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<pgt::PoseGraph>::SharedPtr posegraph_incremental_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  uint16_t robot_id_{0};
  std::string odom_frame_id_;
  std::string base_link_frame_id_;
  std::string map_frame_id_;
  bool publish_lcd_tf_{true};

  std::vector<pgt::PoseGraphEdge> loop_closure_edges_;
  std::vector<pgt::PoseGraphEdge> odometry_edges_;
  std::vector<pgt::PoseGraphEdge> inlier_edges_;
  std::vector<pgt::PoseGraphNode> pose_graph_nodes_;
};

class KimeraBackendPublisher {
 public:
  explicit KimeraBackendPublisher(rclcpp::Node& node)
      : node_(node),
        tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(node)) {
    odom_frame_id_ = getOrDeclare<std::string>("odom_frame_id", "odom");
    base_link_frame_id_ =
        getOrDeclare<std::string>("base_link_frame_id", "base_link_kimera");
    publish_tf_ = getOrDeclare<bool>("publish_tf", true);
    odometry_pub_ =
        node_.create_publisher<nav_msgs::msg::Odometry>("odometry", 1);
  }

  void publishBackendOutput(const VIO::BackendOutput::ConstPtr& output) {
    if (!output) {
      return;
    }
    publishTf(output);
    publishOdometry(output);
  }

 private:
  template <typename T>
  T getOrDeclare(const std::string& name, const T& default_value) {
    if (!node_.has_parameter(name)) {
      node_.declare_parameter<T>(name, default_value);
    }
    return node_.get_parameter(name).get_value<T>();
  }

  void publishTf(const VIO::BackendOutput::ConstPtr& output) {
    if (!publish_tf_) {
      return;
    }

    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = stampFromNs(output->timestamp_);
    odom_tf.header.frame_id = odom_frame_id_;
    odom_tf.child_frame_id = base_link_frame_id_;
    poseToTransform(output->W_State_Blkf_.pose_, &odom_tf.transform);
    tf_broadcaster_->sendTransform(odom_tf);
  }

  void publishOdometry(const VIO::BackendOutput::ConstPtr& output) {
    nav_msgs::msg::Odometry odometry_msg;
    odometry_msg.header.stamp = stampFromNs(output->timestamp_);
    odometry_msg.header.frame_id = odom_frame_id_;
    odometry_msg.child_frame_id = base_link_frame_id_;

    const gtsam::Pose3& pose = output->W_State_Blkf_.pose_;
    poseToMsg(pose, &odometry_msg.pose.pose);

    const gtsam::Matrix3 body_R_world = pose.rotation().transpose();
    const gtsam::Vector3 velocity_body =
        body_R_world * output->W_State_Blkf_.velocity_;
    odometry_msg.twist.twist.linear.x = velocity_body.x();
    odometry_msg.twist.twist.linear.y = velocity_body.y();
    odometry_msg.twist.twist.linear.z = velocity_body.z();

    odometry_pub_->publish(odometry_msg);
  }

  rclcpp::Node& node_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string odom_frame_id_;
  std::string base_link_frame_id_;
  bool publish_tf_{true};
};

class KimeraRos2Visualizer : public VIO::Visualizer3D {
 public:
  explicit KimeraRos2Visualizer(KimeraBackendPublisher& backend_publisher)
      : VIO::Visualizer3D(VIO::VisualizationType::kNone),
        backend_publisher_(backend_publisher) {}

  VIO::VisualizerOutput::UniquePtr spinOnce(
      const VIO::VisualizerInput& input) override {
    if (input.backend_output_) {
      backend_publisher_.publishBackendOutput(input.backend_output_);
    }
    return std::make_unique<VIO::VisualizerOutput>();
  }

 private:
  KimeraBackendPublisher& backend_publisher_;
};

class KimeraRos2Display : public VIO::DisplayBase {
 public:
  KimeraRos2Display() : VIO::DisplayBase(VIO::DisplayType::kOpenCV) {}

  void spinOnce(VIO::DisplayInputBase::UniquePtr&&) override {}
};

class KimeraVioRos2Node : public rclcpp::Node {
 public:
  explicit KimeraVioRos2Node(const rclcpp::NodeOptions& options)
      : Node("mono_hydra_vio_ros_node", options),
        lcd_publisher_(*this),
        backend_publisher_(*this) {
    readParameters();
    setGflagsFromRosParameters();
    createPipeline();
    publishStaticCameraTransforms();
    createSubscriptions();
    pipeline_thread_ = std::thread([this]() {
      if (pipeline_) {
        pipeline_->spin();
      }
    });
    RCLCPP_INFO(get_logger(),
                "Started benchmarked Kimera VIO ROS 2 node with params '%s'.",
                params_folder_path_.c_str());
  }

  ~KimeraVioRos2Node() override {
    shutdown_requested_ = true;
    if (pipeline_ && !pipeline_->isShutdown()) {
      pipeline_->shutdown();
    }
    if (pipeline_thread_.joinable()) {
      pipeline_thread_.join();
    }
  }

 private:
  template <typename T>
  T getOrDeclare(const std::string& name, const T& default_value) {
    if (!has_parameter(name)) {
      declare_parameter<T>(name, default_value);
    }
    return get_parameter(name).get_value<T>();
  }

  void readParameters() {
    params_folder_path_ =
        getOrDeclare<std::string>("params_folder_path", std::string());
    sensor_params_folder_path_ =
        getOrDeclare<std::string>("sensor_params_folder_path", std::string());
    left_topic_ = getOrDeclare<std::string>("left_cam_topic",
                                            "/camera/color/image_raw");
    right_topic_ = getOrDeclare<std::string>("right_cam_topic", std::string());
    depth_topic_ = getOrDeclare<std::string>("depth_cam_topic",
                                             "/camera/depth_cam/image_raw");
    imu_topic_ = getOrDeclare<std::string>("imu_topic", "/imu/aligned");
    external_odom_topic_ =
        getOrDeclare<std::string>("external_odom_topic", "/external_odometry");
    semantic_mask_topic_ =
        getOrDeclare<std::string>("semantic_mask_topic", std::string());
    use_external_odom_ = getOrDeclare<bool>("use_external_odom", false);
    use_lcd_ = getOrDeclare<bool>("use_lcd", true);
    visualize_ = getOrDeclare<bool>("visualize", false);
    use_rviz_ = getOrDeclare<bool>("use_rviz", true);
    viz_type_ = getOrDeclare<int>("viz_type", 1);
    log_output_ = getOrDeclare<bool>("log_output", false);
    lcd_no_optimize_ = getOrDeclare<bool>("lcd_no_optimize", false);
    lcd_no_detection_ = getOrDeclare<bool>("lcd_no_detection", false);
    lcd_disable_stereo_match_depth_check_ =
        getOrDeclare<bool>("lcd_disable_stereo_match_depth_check", false);
    no_incremental_pose_ = getOrDeclare<bool>("no_incremental_pose", false);
    do_coarse_imu_camera_temporal_sync_ =
        getOrDeclare<bool>("do_coarse_imu_camera_temporal_sync", false);
    do_fine_imu_camera_temporal_sync_ =
        getOrDeclare<bool>("do_fine_imu_camera_temporal_sync", false);
    publish_camera_tf_ = getOrDeclare<bool>("publish_camera_tf", true);
    publish_backend_state_ =
        getOrDeclare<bool>("publish_backend_state", true);
    base_link_frame_id_ =
        getOrDeclare<std::string>("base_link_frame_id", "base_link_kimera");
    left_cam_frame_id_ =
        getOrDeclare<std::string>("left_cam_frame_id", "left_cam_kimera");
    right_cam_frame_id_ =
        getOrDeclare<std::string>("right_cam_frame_id", "right_cam_kimera");
    force_same_image_timestamp_ =
        getOrDeclare<bool>("force_same_image_timestamp", true);
    semantic_mask_is_label_image_ =
        getOrDeclare<bool>("semantic_mask_is_label_image", true);
    semantic_mask_inflate_px_ =
        std::max(0, getOrDeclare<int>("semantic_mask_inflate_px", 0));
    use_semantic_masking_ = getOrDeclare<bool>("use_semantic_masking", true);
    rgbd_sync_queue_size_ =
        std::max(1, getOrDeclare<int>("rgbd_sync_queue_size", 10));
    stereo_sync_queue_size_ =
        std::max(1, getOrDeclare<int>("stereo_sync_queue_size", 10));
    dynamic_labels_ = getOrDeclare<std::vector<int64_t>>(
        "semantic_dynamic_labels", std::vector<int64_t>{19, 20});

    if (params_folder_path_.empty()) {
      throw std::runtime_error("params_folder_path is required.");
    }
  }

  void setGflagsFromRosParameters() {
    FLAGS_use_external_odometry = use_external_odom_;
    FLAGS_use_lcd = use_lcd_;
    FLAGS_visualize = publish_backend_state_ || (visualize_ && !use_rviz_);
    FLAGS_viz_type = viz_type_;
    FLAGS_log_output = log_output_;
    FLAGS_lcd_no_optimize = lcd_no_optimize_;
    FLAGS_lcd_no_detection = lcd_no_detection_;
    FLAGS_lcd_disable_stereo_match_depth_check =
        lcd_disable_stereo_match_depth_check_;
    FLAGS_no_incremental_pose = no_incremental_pose_;
    FLAGS_do_coarse_imu_camera_temporal_sync =
        do_coarse_imu_camera_temporal_sync_;
    FLAGS_do_fine_imu_camera_temporal_sync =
        do_fine_imu_camera_temporal_sync_;
  }

  void createPipeline() {
    if (sensor_params_folder_path_.empty()) {
      vio_params_ = std::make_shared<VIO::VioParams>(params_folder_path_);
    } else {
      vio_params_ = std::make_shared<VIO::VioParams>(
          params_folder_path_, sensor_params_folder_path_);
    }

    std::unique_ptr<VIO::PreloadedVocab> preloaded_vocab;
    if (FLAGS_use_lcd) {
      preloaded_vocab = std::make_unique<VIO::PreloadedVocab>();
    }

    std::unique_ptr<VIO::Visualizer3D> visualizer;
    std::unique_ptr<VIO::DisplayBase> display;
    if (publish_backend_state_) {
      visualizer = std::make_unique<KimeraRos2Visualizer>(backend_publisher_);
      display = std::make_unique<KimeraRos2Display>();
    }

    switch (vio_params_->frontend_type_) {
      case VIO::FrontendType::kMonoImu:
        pipeline_ = std::make_unique<VIO::MonoImuPipeline>(
            *vio_params_, std::move(visualizer), std::move(display),
            std::move(preloaded_vocab));
        break;
      case VIO::FrontendType::kStereoImu:
        pipeline_ = std::make_unique<VIO::StereoImuPipeline>(
            *vio_params_, std::move(visualizer), std::move(display),
            std::move(preloaded_vocab));
        break;
      case VIO::FrontendType::kRgbdImu:
        pipeline_ = std::make_unique<VIO::RgbdImuPipeline>(
            *vio_params_, std::move(visualizer), std::move(display),
            std::move(preloaded_vocab));
        break;
      default:
        throw std::runtime_error("Unsupported Kimera VIO frontend_type.");
    }

    pipeline_->registerLcdOutputCallback(
        [this](const VIO::LcdOutput::ConstPtr& msg) {
          if (msg) {
            lcd_publisher_.publishLcdOutput(msg);
          }
        });
  }

  void publishStaticCameraTransforms() {
    if (!publish_camera_tf_ || !vio_params_ || left_cam_frame_id_.empty()) {
      return;
    }

    static_tf_broadcaster_ =
        std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.reserve(vio_params_->camera_params_.size());

    geometry_msgs::msg::TransformStamped left_tf;
    left_tf.header.stamp = now();
    left_tf.header.frame_id = base_link_frame_id_;
    left_tf.child_frame_id = left_cam_frame_id_;
    poseToTransform(vio_params_->camera_params_.at(0).body_Pose_cam_,
                    &left_tf.transform);
    transforms.push_back(left_tf);

    if (vio_params_->camera_params_.size() > 1 && !right_cam_frame_id_.empty()) {
      geometry_msgs::msg::TransformStamped right_tf;
      right_tf.header.stamp = left_tf.header.stamp;
      right_tf.header.frame_id = base_link_frame_id_;
      right_tf.child_frame_id = right_cam_frame_id_;
      poseToTransform(vio_params_->camera_params_.at(1).body_Pose_cam_,
                      &right_tf.transform);
      transforms.push_back(right_tf);
    }

    static_tf_broadcaster_->sendTransform(transforms);
    RCLCPP_INFO(get_logger(),
                "Published Kimera camera static TF from %s to %s.",
                base_link_frame_id_.c_str(),
                left_cam_frame_id_.c_str());
  }

  void createSubscriptions() {
    rclcpp::SensorDataQoS sensor_qos;
    imu_sub_ = create_subscription<ImuMsg>(
        imu_topic_, sensor_qos,
        std::bind(&KimeraVioRos2Node::imuCallback, this,
                  std::placeholders::_1));

    if (use_external_odom_ && !external_odom_topic_.empty()) {
      external_odom_sub_ = create_subscription<OdometryMsg>(
          external_odom_topic_, rclcpp::QoS(200),
          std::bind(&KimeraVioRos2Node::externalOdomCallback, this,
                    std::placeholders::_1));
    }

    if (use_semantic_masking_ && !semantic_mask_topic_.empty()) {
      semantic_mask_sub_ = create_subscription<ImageMsg>(
          semantic_mask_topic_, sensor_qos,
          std::bind(&KimeraVioRos2Node::semanticMaskCallback, this,
                    std::placeholders::_1));
    }

    if (vio_params_->frontend_type_ == VIO::FrontendType::kRgbdImu) {
      rgbd_left_sub_.subscribe(this, left_topic_, rmw_qos_profile_sensor_data);
      rgbd_depth_sub_.subscribe(this, depth_topic_, rmw_qos_profile_sensor_data);
      rgbd_sync_ = std::make_shared<message_filters::Synchronizer<RgbdSyncPolicy>>(
          RgbdSyncPolicy(rgbd_sync_queue_size_), rgbd_left_sub_, rgbd_depth_sub_);
      rgbd_sync_->registerCallback(std::bind(
          &KimeraVioRos2Node::rgbdCallback, this, std::placeholders::_1,
          std::placeholders::_2));
    } else if (vio_params_->frontend_type_ == VIO::FrontendType::kStereoImu) {
      stereo_left_sub_.subscribe(this, left_topic_, rmw_qos_profile_sensor_data);
      stereo_right_sub_.subscribe(
          this, right_topic_, rmw_qos_profile_sensor_data);
      stereo_sync_ =
          std::make_shared<message_filters::Synchronizer<StereoSyncPolicy>>(
              StereoSyncPolicy(stereo_sync_queue_size_), stereo_left_sub_, stereo_right_sub_);
      stereo_sync_->registerCallback(std::bind(
          &KimeraVioRos2Node::stereoCallback, this, std::placeholders::_1,
          std::placeholders::_2));
    } else {
      mono_image_sub_ = create_subscription<ImageMsg>(
          left_topic_, sensor_qos,
          std::bind(&KimeraVioRos2Node::monoImageCallback, this,
                    std::placeholders::_1));
    }
  }

  cv::Mat readImage(const ImagePtr& msg) const {
    CHECK(msg);
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg);
    const cv::Mat& image = cv_ptr->image;
    if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
      cv::Mat gray;
      cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
      return gray;
    }
    if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
      cv::Mat gray;
      cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);
      return gray;
    }
    if (msg->encoding == sensor_msgs::image_encodings::BGRA8) {
      cv::Mat gray;
      cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
      return gray;
    }
    CHECK(cv_ptr->encoding == sensor_msgs::image_encodings::MONO8 ||
          cv_ptr->encoding == sensor_msgs::image_encodings::TYPE_8UC1)
        << "Expected MONO8, 8UC1, BGR8, RGB8, or BGRA8 image encoding.";
    return image.clone();
  }

  cv::Mat readDepthImage(const ImagePtr& msg) const {
    CHECK(msg);
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg);
    cv::Mat depth = cv_ptr->image;
    CHECK_EQ(depth.channels(), 1) << "Depth image must have one channel.";
    if (depth.type() != CV_32FC1 && depth.type() != CV_16UC1) {
      depth.convertTo(depth, CV_32FC1);
    }
    return depth.clone();
  }

  void attachLatestSemanticMask(VIO::Frame* frame) const {
    if (!frame || !use_semantic_masking_ || !has_semantic_mask_) {
      return;
    }
    cv::Mat mask = latest_semantic_mask_;
    if (mask.size() != frame->img_.size()) {
      cv::resize(mask, mask, frame->img_.size(), 0, 0, cv::INTER_NEAREST);
    }
    frame->detection_mask_ = mask.clone();
  }

  void monoImageCallback(const ImagePtr msg) {
    if (shutdown_requested_ || !pipeline_) {
      return;
    }
    const auto frame_id = frame_count_++;
    auto frame = std::make_unique<VIO::Frame>(
        frame_id, stampToNs(msg->header.stamp), vio_params_->camera_params_.at(0),
        readImage(msg));
    attachLatestSemanticMask(frame.get());
    pipeline_->fillLeftFrameQueue(std::move(frame));
  }

  void stereoCallback(const ImagePtr left_msg, const ImagePtr right_msg) {
    if (shutdown_requested_ || !pipeline_) {
      return;
    }
    auto stereo_pipeline = dynamic_cast<VIO::StereoImuPipeline*>(pipeline_.get());
    CHECK(stereo_pipeline);
    const auto frame_id = frame_count_++;
    const VIO::Timestamp left_stamp = stampToNs(left_msg->header.stamp);
    const VIO::Timestamp right_stamp =
        force_same_image_timestamp_ ? left_stamp : stampToNs(right_msg->header.stamp);
    auto left = std::make_unique<VIO::Frame>(
        frame_id, left_stamp, vio_params_->camera_params_.at(0), readImage(left_msg));
    attachLatestSemanticMask(left.get());
    pipeline_->fillLeftFrameQueue(std::move(left));
    stereo_pipeline->fillRightFrameQueue(std::make_unique<VIO::Frame>(
        frame_id, right_stamp, vio_params_->camera_params_.at(1),
        readImage(right_msg)));
  }

  void rgbdCallback(const ImagePtr color_msg, const ImagePtr depth_msg) {
    if (shutdown_requested_ || !pipeline_) {
      return;
    }
    auto rgbd_pipeline = dynamic_cast<VIO::RgbdImuPipeline*>(pipeline_.get());
    CHECK(rgbd_pipeline);
    const auto frame_id = frame_count_++;
    const VIO::Timestamp color_stamp = stampToNs(color_msg->header.stamp);
    const VIO::Timestamp depth_stamp =
        force_same_image_timestamp_ ? color_stamp : stampToNs(depth_msg->header.stamp);

    auto frame = std::make_unique<VIO::Frame>(
        frame_id, color_stamp, vio_params_->camera_params_.at(0),
        readImage(color_msg));
    attachLatestSemanticMask(frame.get());
    pipeline_->fillLeftFrameQueue(std::move(frame));
    rgbd_pipeline->fillDepthFrameQueue(std::make_unique<VIO::DepthFrame>(
        frame_id, depth_stamp, readDepthImage(depth_msg)));
  }

  void imuCallback(const ImuMsg::ConstSharedPtr msg) {
    if (shutdown_requested_ || !pipeline_) {
      return;
    }
    VIO::ImuAccGyr imu_accgyr;
    imu_accgyr(0) = msg->linear_acceleration.x;
    imu_accgyr(1) = msg->linear_acceleration.y;
    imu_accgyr(2) = msg->linear_acceleration.z;
    imu_accgyr(3) = msg->angular_velocity.x;
    imu_accgyr(4) = msg->angular_velocity.y;
    imu_accgyr(5) = msg->angular_velocity.z;
    pipeline_->fillSingleImuQueue(
        VIO::ImuMeasurement(stampToNs(msg->header.stamp), imu_accgyr));
  }

  void externalOdomCallback(const OdometryMsg::ConstSharedPtr msg) {
    if (shutdown_requested_ || !pipeline_) {
      return;
    }
    const gtsam::Pose3 pose = odometryToPose(*msg);
    const auto& linear = msg->twist.twist.linear;
    const gtsam::Vector3 body_vel_body(linear.x, linear.y, linear.z);
    const gtsam::Vector3 body_vel_world = pose.rotation() * body_vel_body;
    pipeline_->fillExternalOdomQueue(VIO::ExternalOdomMeasurement(
        stampToNs(msg->header.stamp), gtsam::NavState(pose, body_vel_world)));
  }

  void semanticMaskCallback(const ImageMsg::ConstSharedPtr msg) {
    try {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(
          msg, sensor_msgs::image_encodings::TYPE_8UC1);
      cv::Mat mask = cv_ptr->image.clone();
      if (semantic_mask_is_label_image_) {
        cv::Mat keep(mask.size(), CV_8UC1, cv::Scalar(255));
        for (const auto label : dynamic_labels_) {
          keep.setTo(0, mask == static_cast<int>(label));
        }
        mask = keep;
      } else {
        double min_val = 0.0;
        double max_val = 0.0;
        cv::minMaxLoc(mask, &min_val, &max_val);
        if (max_val <= 1.0) {
          mask *= 255;
        }
      }
      cv::threshold(mask, mask, 0, 255, cv::THRESH_BINARY);
      if (semantic_mask_inflate_px_ > 0) {
        cv::Mat rejected_mask;
        cv::bitwise_not(mask, rejected_mask);
        const int kernel_size = 2 * semantic_mask_inflate_px_ + 1;
        cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
        cv::dilate(rejected_mask, rejected_mask, kernel);
        cv::bitwise_not(rejected_mask, mask);
      }
      latest_semantic_mask_ = mask;
      has_semantic_mask_ = true;
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_WARN(get_logger(), "cv_bridge semantic mask error: %s", e.what());
      has_semantic_mask_ = false;
    }
  }

  KimeraLcdPublisher lcd_publisher_;
  KimeraBackendPublisher backend_publisher_;
  VIO::VioParams::Ptr vio_params_;
  VIO::Pipeline::UniquePtr pipeline_;
  std::thread pipeline_thread_;
  std::atomic_bool shutdown_requested_{false};

  std::string params_folder_path_;
  std::string sensor_params_folder_path_;
  std::string left_topic_;
  std::string right_topic_;
  std::string depth_topic_;
  std::string imu_topic_;
  std::string external_odom_topic_;
  std::string semantic_mask_topic_;
  bool use_external_odom_{false};
  bool use_lcd_{true};
  bool use_rviz_{true};
  bool visualize_{false};
  bool log_output_{false};
  bool lcd_no_optimize_{false};
  bool lcd_no_detection_{false};
  bool lcd_disable_stereo_match_depth_check_{false};
  bool no_incremental_pose_{false};
  bool do_coarse_imu_camera_temporal_sync_{false};
  bool do_fine_imu_camera_temporal_sync_{false};
  bool publish_camera_tf_{true};
  bool publish_backend_state_{true};
  int viz_type_{1};
  std::string base_link_frame_id_;
  std::string left_cam_frame_id_;
  std::string right_cam_frame_id_;
  bool force_same_image_timestamp_{true};
  bool semantic_mask_is_label_image_{true};
  int semantic_mask_inflate_px_{0};
  bool use_semantic_masking_{true};
  int rgbd_sync_queue_size_{10};
  int stereo_sync_queue_size_{10};
  std::vector<int64_t> dynamic_labels_;
  VIO::FrameId frame_count_{0};

  cv::Mat latest_semantic_mask_;
  bool has_semantic_mask_{false};

  rclcpp::Subscription<ImuMsg>::SharedPtr imu_sub_;
  rclcpp::Subscription<OdometryMsg>::SharedPtr external_odom_sub_;
  rclcpp::Subscription<ImageMsg>::SharedPtr mono_image_sub_;
  rclcpp::Subscription<ImageMsg>::SharedPtr semantic_mask_sub_;
  message_filters::Subscriber<ImageMsg> rgbd_left_sub_;
  message_filters::Subscriber<ImageMsg> rgbd_depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<RgbdSyncPolicy>> rgbd_sync_;
  message_filters::Subscriber<ImageMsg> stereo_left_sub_;
  message_filters::Subscriber<ImageMsg> stereo_right_sub_;
  std::shared_ptr<message_filters::Synchronizer<StereoSyncPolicy>> stereo_sync_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

}  // namespace mono_hydra_vio_ros2

int main(int argc, char* argv[]) {
  auto non_ros_args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  std::vector<char*> gflags_argv;
  gflags_argv.reserve(non_ros_args.size());
  for (auto& arg : non_ros_args) {
    gflags_argv.push_back(arg.data());
  }
  int gflags_argc = static_cast<int>(gflags_argv.size());
  char** gflags_argv_ptr = gflags_argv.data();
  google::ParseCommandLineFlags(&gflags_argc, &gflags_argv_ptr, true);
  google::InitGoogleLogging(argv[0]);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<mono_hydra_vio_ros2::KimeraVioRos2Node>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
