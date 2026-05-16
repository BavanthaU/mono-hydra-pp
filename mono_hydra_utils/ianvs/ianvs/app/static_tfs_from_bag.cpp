#include <yaml-cpp/yaml.h>

#include <filesystem>

#include <CLI/CLI.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "ianvs/detail/frame_remapper.h"

struct AppArgs {
  void add_to_app(CLI::App& app);

  std::filesystem::path bag;
  std::filesystem::path output;
  ianvs::FrameRemapper::Config remapper;
};

void AppArgs::add_to_app(CLI::App& app) {
  app.add_option("bag_path", bag)->required()->check(CLI::ExistingPath)->description("Bag to open");
  app.add_option("-o,--output", output);
  app.add_option("-p,--prefix-frames", remapper.prefix)
      ->take_last()
      ->description("prefix to apply to ALL frames");
  app.add_option("-f,--filter-frames", remapper.filter)
      ->join('|')
      ->description("optional regex filter to drop frames (applied before prefix)");
  app.add_option("-k,--keep-frames", remapper.keep)
      ->join('|')
      ->description("optional regex filter to keep frames (applied before prefix)");
  app.add_option("-s,--frame-substitution", remapper.substitutions)
      ->description("apply substitution to frames (match and substituion are separated by :)");
  app.footer(R"(
Both --filter-frames and --keep-frames support two match modes for a given
TF. They can either be provided with a single regex that can match either
the parent or the child frame OR they can be provided with two regexes in
the form <PARENT_REGEX>:<CHILD_REGEX>, which requires that the parent frame
matches the first regex and the child frame matches the second regex. This
precludes the use of non-captured groups in the regex
)");
}

int main(int argc, char** argv) {
  CLI::App app("Utility to save static TFS from a file");
  argv = app.ensure_utf8(argv);
  app.allow_extras();
  app.get_formatter()->column_width(50);

  AppArgs args;
  args.add_to_app(app);
  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    return app.exit(e);
  }

  args.bag = std::filesystem::canonical(args.bag);
  rosbag2_storage::StorageOptions storage_opts;
  storage_opts.uri = args.bag;
  auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(storage_opts);
  if (!reader) {
    return 1;
  }

  ianvs::FrameRemapper::PoseMap pose_map;
  const ianvs::FrameRemapper remapper(args.remapper);

  rosbag2_storage::StorageFilter filter;
  filter.topics = {"/tf_static"};
  const rclcpp::Serialization<tf2_msgs::msg::TFMessage> serialization;

  reader->open(storage_opts);
  reader->set_filter(filter);
  while (reader->has_next()) {
    const auto msg = reader->read_next();
    rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
    auto tf_msg = std::make_shared<tf2_msgs::msg::TFMessage>();
    serialization.deserialize_message(&serialized_msg, tf_msg.get());
    remapper.updatePoseMap(*tf_msg, pose_map);
  }

  YAML::Node root;
  for (const auto& [parent, children] : pose_map) {
    for (const auto& [child, pose] : children) {
      YAML::Node tf;
      tf["frame_id"] = parent;
      tf["child_frame_id"] = child;
      tf["x"] = pose.transform.translation.x;
      tf["y"] = pose.transform.translation.y;
      tf["z"] = pose.transform.translation.z;
      tf["qw"] = pose.transform.rotation.w;
      tf["qx"] = pose.transform.rotation.x;
      tf["qy"] = pose.transform.rotation.y;
      tf["qz"] = pose.transform.rotation.z;
      root["frames"].push_back(tf);
    }
  }

  std::filesystem::path output = args.output;
  if (output.empty()) {
    output = args.bag.parent_path() / (args.bag.stem().string() + "_static_tfs.yaml");
  }

  std::cout << "Saving " << root["frames"].size() << "transforms to " << output << std::endl;
  std::ofstream fout(output);
  fout << root;
  return 0;
}
