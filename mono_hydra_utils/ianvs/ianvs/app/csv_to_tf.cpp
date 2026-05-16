#include <tf2_ros/transform_broadcaster.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <CLI/CLI.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

using geometry_msgs::msg::TransformStamped;

class CsvToTfNode : public rclcpp::Node {
 public:
  explicit CsvToTfNode(const std::vector<TransformStamped>& transforms);

  const std::vector<geometry_msgs::msg::TransformStamped> transforms_;

 private:
  void callback();
  rclcpp::Time get_curr_tf_stamp() const;
  bool curr_tf_is_stale(const rclcpp::Time& ros_stamp) const;

 private:
  size_t idx_;

  double offset_;
  double lookahead_;
  double stale_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

CsvToTfNode::CsvToTfNode(const std::vector<TransformStamped>& transforms)
    : rclcpp::Node("csv_to_tf"), transforms_(transforms), idx_(0) {
  broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  offset_ = declare_parameter<double>("offset_s", 0.0);
  lookahead_ = declare_parameter<double>("lookahead_s", 0.0);
  stale_ = declare_parameter<double>("stale_threshold_s", 0.5);

  auto timer_callback = [this]() { callback(); };
  const auto period = declare_parameter<double>("poll_period_s", 0.001);
  timer_ = create_wall_timer(std::chrono::duration<double>(period), timer_callback);

  auto logger = get_logger();
  RCLCPP_INFO_STREAM(logger, "Polling clock every " << period << " [s]");
  RCLCPP_INFO_STREAM(logger, "Offsetting transforms by " << offset_ << " [s]");
  RCLCPP_INFO_STREAM(logger, "Using lookahead of " << lookahead_ << " [s]");
  RCLCPP_INFO_STREAM(logger, "Discarding TFs more than " << stale_ << " [s] behind clock");
}

void CsvToTfNode::callback() {
  auto logger = get_logger();
  const auto stamp = get_clock()->now();
  const auto stamp_ns = stamp.nanoseconds();

  const auto prev_idx = idx_;
  while (curr_tf_is_stale(stamp)) {
    const auto curr_ns = get_curr_tf_stamp().nanoseconds();
    RCLCPP_DEBUG_STREAM(logger, "Dropping " << curr_ns << " [ns] @ " << stamp_ns << " [ns]");
    ++idx_;
  }

  size_t num_dropped = idx_ - prev_idx;
  if (num_dropped > 0) {
    RCLCPP_WARN_STREAM(logger, "Dropped " << num_dropped << " older than " << stamp_ns << " [ns]");
  }

  if (idx_ >= transforms_.size()) {
    RCLCPP_INFO(logger, "Finished publishing transforms");
    timer_->cancel();
    return;
  }

  const auto next_stamp = get_curr_tf_stamp();
  const auto next_ns = next_stamp.nanoseconds();
  const auto diff = next_stamp - stamp;
  const auto diff_ns = diff.nanoseconds();
  RCLCPP_DEBUG_STREAM(logger, "Waiting for " << next_ns << " [ns] (diff: " << diff_ns << ")");
  if (diff > rclcpp::Duration::from_seconds(lookahead_)) {
    return;
  }

  RCLCPP_DEBUG_STREAM(logger, "Sending " << next_ns << " [ns] @ " << stamp_ns << " [ns]");
  auto msg = transforms_[idx_];
  msg.header.stamp = next_stamp;
  broadcaster_->sendTransform(msg);
  ++idx_;
}

rclcpp::Time CsvToTfNode::get_curr_tf_stamp() const {
  if (idx_ >= transforms_.size()) {
    return rclcpp::Time();
  }

  return rclcpp::Time(transforms_[idx_].header.stamp) + rclcpp::Duration::from_seconds(offset_);
}

bool CsvToTfNode::curr_tf_is_stale(const rclcpp::Time& ros_stamp) const {
  const auto curr_stamp = get_curr_tf_stamp();
  if (curr_stamp.nanoseconds() == 0 || ros_stamp < curr_stamp) {
    return false;
  }

  return (ros_stamp - curr_stamp) > rclcpp::Duration::from_seconds(stale_);
}

struct ParseOptions {
  std::filesystem::path filepath;

  bool skip_first = true;
  std::string parent = "odom";
  std::string child = "base_link";
  std::vector<std::string> parse_order = {"x", "y", "z", "qw", "qx", "qy", "qz"};
  std::vector<double> origin = {0.0, 0.0, 0.0};

  void add_args(CLI::App& app);
  TransformStamped parse_line(const std::string& line) const;
  std::vector<TransformStamped> parse() const;
};

void ParseOptions::add_args(CLI::App& app) {
  app.add_option("filepath", filepath)
      ->check(CLI::ExistingFile)
      ->required()
      ->description("Path to CSV trajectory file");
  app.add_option("--parent", parent, "Frame ID for parent frame")->default_val("odom");
  app.add_option("--child", child, "Frame ID for child frame")->default_val("base_link");
  app.add_option("--origin", origin, "Origin offset")->expected(3);
  app.add_option("--order", parse_order, "Column order")->expected(7);
  app.add_flag("--skip-first/!--no-skip-first", skip_first, "Parse first line as data");
}

TransformStamped ParseOptions::parse_line(const std::string& line) const {
  std::istringstream iss(line);

  TransformStamped tf;
  tf.header.frame_id = parent;
  tf.child_frame_id = child;

  size_t index = 0;
  std::string token;
  std::map<std::string, double> elements;
  while (std::getline(iss, token, ',')) {
    if (index == 0) {
      tf.header.stamp = rclcpp::Time(std::stoll(token));
      ++index;
      continue;
    }

    elements[parse_order.at(index - 1)] = std::stod(token);
    ++index;
  }

  tf.transform.translation.x = elements.at("x") - origin[0];
  tf.transform.translation.y = elements.at("y") - origin[1];
  tf.transform.translation.z = elements.at("z") - origin[2];
  tf.transform.rotation.w = elements.at("qw");
  tf.transform.rotation.x = elements.at("qx");
  tf.transform.rotation.y = elements.at("qy");
  tf.transform.rotation.z = elements.at("qz");
  return tf;
}

std::vector<TransformStamped> ParseOptions::parse() const {
  std::ifstream file(filepath);
  if (!file) {
    return {};
  }

  std::string line;
  bool first_line = true;
  std::vector<TransformStamped> transforms;
  while (std::getline(file, line)) {
    if (first_line) {
      first_line = false;
      if (skip_first) {
        continue;
      }
    }

    transforms.push_back(parse_line(line));
  }

  return transforms;
}

std::vector<char*> get_ros_args(int& argc, char** argv) {
  // filters argc and argv to only have wrapper node args
  std::vector<char*> ros_argv;
  ros_argv.push_back(argv[0]);

  bool found_ros_args = false;
  const int max_args = argc;
  for (int i = 1; i < max_args; ++i) {
    std::string arg(argv[i]);
    if (arg == "--ros-args") {
      argc = i;
      found_ros_args = true;
    }

    if (found_ros_args) {
      ros_argv.push_back(argv[i]);
    }
  }

  return ros_argv;
}

int main(int argc, char* argv[]) {
  CLI::App app("Node publishing parent_T_child from CSV");
  app.allow_extras();
  app.get_formatter()->column_width(50);

  ParseOptions opts;
  opts.add_args(app);
  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    return app.exit(e);
  }

  const auto transforms = opts.parse();
  const auto ros_argv = get_ros_args(argc, argv);
  rclcpp::init(ros_argv.size(), ros_argv.data());
  auto node = std::make_shared<CsvToTfNode>(transforms);
  RCLCPP_INFO_STREAM(node->get_logger(), "Publishing " << opts.parent << "_T_" << opts.child);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
