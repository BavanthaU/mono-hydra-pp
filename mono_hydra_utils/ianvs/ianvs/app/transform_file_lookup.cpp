#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <iomanip>

#include <CLI/CLI.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/buffer_core.hpp>

using geometry_msgs::msg::TransformStamped;

enum class FormatEnum {
  //! Show transform in order X Y Z QX QY QZ QW
  FLAT,
  //! Show homogeneous transformation matrix
  HOMOGENEOUS,
  //! Show JSON representation of transform
  JSON,
  //! Show args for static transform publisher
  ARGS,
};

struct AppArgs {
  std::filesystem::path tf_filepath;
  std::string from_frame;
  std::string to_frame;
  bool set_precision = true;
  size_t precision = 3;
  size_t column_width = 4;
  FormatEnum tf_format = FormatEnum::JSON;

  void add_to_app(CLI::App& app);
};

void AppArgs::add_to_app(CLI::App& app) {
  app.add_option("tf_filepath", tf_filepath)
      ->required()
      ->description("File to read")
      ->check(CLI::ExistingFile);
  app.add_option("from_frame", from_frame)->required()->description("from in to_T_from");
  app.add_option("to_frame", to_frame)->required()->description("to in to_T_from");
  app.add_flag("--set-precision,!--no-set-precision", set_precision)
      ->description("set output stream precision");
  app.add_option("--precision,-p", precision)
      ->description("output stream precision")
      ->check(CLI::PositiveNumber);
  app.add_option("--column-width,-w", column_width)
      ->description("matrix column width when printing")
      ->check(CLI::PositiveNumber);

  const static std::map<std::string, FormatEnum> format_names{{"flat", FormatEnum::FLAT},
                                                              {"matrix", FormatEnum::HOMOGENEOUS},
                                                              {"json", FormatEnum::JSON},
                                                              {"args", FormatEnum::ARGS}};
  app.add_option("-f,--format", tf_format, "output string format")
      ->transform(CLI::CheckedTransformer(format_names, CLI::ignore_case));
}

void loadTransformsFromFile(const std::filesystem::path& filepath, tf2::BufferCore& buffer) {
  if (!std::filesystem::exists(filepath)) {
    std::cerr << "Invalid filepath: " << filepath << std::endl;
    return;
  }

  const auto node = YAML::LoadFile(filepath);
  for (const auto& tf : node["frames"]) {
    try {
      std::string frame_id = tf["frame_id"].as<std::string>();
      std::string child_id = tf["child_frame_id"].as<std::string>();

      TransformStamped msg;
      msg.header.frame_id = frame_id;
      msg.child_frame_id = child_id;
      msg.transform.translation.x = tf["x"].as<double>();
      msg.transform.translation.y = tf["y"].as<double>();
      msg.transform.translation.z = tf["z"].as<double>();
      msg.transform.rotation.w = tf["qw"].as<double>();
      msg.transform.rotation.x = tf["qx"].as<double>();
      msg.transform.rotation.y = tf["qy"].as<double>();
      msg.transform.rotation.z = tf["qz"].as<double>();
      buffer.setTransform(msg, "file", true);
    } catch (std::exception& e) {
      std::cerr << "Failed to parse " << tf << ": " << e.what() << std::endl;
      continue;
    }
  }
}

std::string getFrameList(const tf2::BufferCore& buffer) {
  const auto all_frames = buffer.getAllFrameNames();
  std::stringstream ss;
  for (const auto& frame : all_frames) {
    ss << " - " << frame << "\n";
  }

  return ss.str();
}

std::string transformToMat(const AppArgs& args,
                           const geometry_msgs::msg::Quaternion& rot,
                           const geometry_msgs::msg::Vector3& pos) {
  std::stringstream ss;
  size_t width = args.column_width;
  if (args.set_precision) {
    ss << std::fixed << std::setprecision(args.precision);
    width += args.precision;
  }

  tf2::Quaternion q(rot.x, rot.y, rot.z, rot.w);
  tf2::Matrix3x3 mat(q);
  ss << "[[" << std::setw(width) << mat[0][0] << ", " << std::setw(width) << mat[0][1] << ", "
     << std::setw(width) << mat[0][2] << ", " << std::setw(width) << pos.x << "],\n";
  ss << " [" << std::setw(width) << mat[1][0] << ", " << std::setw(width) << mat[1][1] << ", "
     << std::setw(width) << mat[1][2] << ", " << std::setw(width) << pos.y << "],\n";
  ss << " [" << std::setw(width) << mat[2][0] << ", " << std::setw(width) << mat[2][1] << ", "
     << std::setw(width) << mat[2][2] << ", " << std::setw(width) << pos.z << "],\n";
  ss << " [" << std::setw(width) << 0.0 << ", " << std::setw(width) << 0.0 << ", "
     << std::setw(width) << 0.0 << ", " << std::setw(width) << 1.0 << "]]";
  return ss.str();
}

std::string showTransform(const TransformStamped& tf, const AppArgs& args) {
  std::stringstream ss;
  const auto from = tf.child_frame_id;
  const auto to = tf.header.frame_id;
  const auto& pos = tf.transform.translation;
  const auto& rot = tf.transform.rotation;
  if (args.set_precision) {
    ss << std::fixed << std::setprecision(args.precision);
  }

  switch (args.tf_format) {
    case FormatEnum::FLAT:
      ss << pos.x << " " << pos.y << " " << pos.z << " " << rot.x << " " << rot.y << " " << rot.z
         << " " << rot.w;
      break;
    case FormatEnum::HOMOGENEOUS:
      ss << transformToMat(args, rot, pos);
      break;
    case FormatEnum::JSON:
      ss << "{'" << to << "_T_" << from << "': {'pos': [" << pos.x << ", " << pos.y << ", " << pos.z
         << "], 'rot': {'w': " << rot.w << ", 'x': " << rot.x << ", 'y': " << rot.y
         << ", 'z': " << rot.z << "}}";
      break;
    case FormatEnum::ARGS:
      ss << "--frame-id " << to << " --child-frame-id " << from << " --x " << pos.x << " --y "
         << pos.y << " --z " << pos.z << " --qw " << rot.w << " --qx " << rot.x << " --qy " << rot.y
         << " --qz " << rot.z;
      break;
  }

  return ss.str();
}

int main(int argc, char** argv) {
  CLI::App app("Utility that looks up transform between two frames from a file");
  app.get_formatter()->column_width(50);

  AppArgs args;
  args.add_to_app(app);
  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    return app.exit(e);
  }

  tf2::BufferCore buffer;
  loadTransformsFromFile(args.tf_filepath, buffer);
  std::string error;
  tf2::TimePoint stamp;
  if (!buffer.canTransform(args.to_frame, args.from_frame, stamp, &error)) {
    std::cerr << "Cannot find " << args.to_frame << "_T_" << args.from_frame << ": " << error
              << std::endl;

    std::cerr << "Available frames:\n" << getFrameList(buffer) << std::endl;
    return 1;
  }

  const auto tf = buffer.lookupTransform(args.to_frame, args.from_frame, stamp);
  std::cout << showTransform(tf, args) << std::endl;
  return 0;
}
