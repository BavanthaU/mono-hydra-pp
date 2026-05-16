#include <filesystem>
#include <map>
#include <set>
#include <vector>

#include <CLI/CLI.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>

#include "ianvs/detail/string_transforms.h"

using rosbag2_transport::ReaderWriterFactory;

struct MessageInfo {
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg;
  const rosbag2_storage::TopicMetadata* metadata = nullptr;

  operator bool() const { return msg != nullptr && metadata != nullptr; }
};

struct ReaderInfo {
  ReaderInfo(const std::filesystem::path& bagpath) {
    rosbag2_storage::StorageOptions opts;
    opts.uri = bagpath;
    reader = ReaderWriterFactory::make_reader(opts);
    if (!reader) {
      return;
    }

    reader->open(opts);

    const auto metadata = reader->get_all_topics_and_types();
    for (const auto& data : metadata) {
      lookup[data.name] = data;
    }
  }

  operator bool() const { return reader != nullptr; }

  MessageInfo next() const {
    while (reader->has_next()) {
      auto msg = reader->read_next();
      if (!msg) {
        continue;
      }

      auto iter = lookup.find(msg->topic_name);
      if (iter == lookup.end()) {
        std::cout << "Error: could not find metadata for topic '" << msg->topic_name << "'"
                  << std::endl;
        continue;
      }

      return {msg, &iter->second};
    }

    return {};
  }

  std::unique_ptr<rosbag2_cpp::Reader> reader;
  std::map<std::string, rosbag2_storage::TopicMetadata> lookup;
};

using ReaderVec = std::vector<ReaderInfo>;
using MsgVec = std::vector<MessageInfo>;

void fill_messages(const ReaderVec& readers, MsgVec& msgs) {
  for (size_t i = 0; i < readers.size(); ++i) {
    if (!readers[i]) {
      continue;
    }

    if (msgs[i]) {
      continue;
    }

    msgs[i] = readers[i].next();
  }
}

MessageInfo get_next_message(MsgVec& msgs) {
  size_t best_idx = 0;
  MessageInfo best_info;
  for (size_t i = 0; i < msgs.size(); ++i) {
    const auto& info = msgs[i];
    if (!info) {
      continue;
    }

    if (!best_info || info.msg->recv_timestamp < best_info.msg->recv_timestamp) {
      best_info = info;
      best_idx = i;
    }
  }

  if (best_info) {
    msgs[best_idx] = {};
  }

  return best_info;
}

std::string print_bag_list(const std::vector<std::filesystem::path>& paths) {
  std::stringstream ss;
  ss << "[";
  auto iter = paths.begin();
  while (iter != paths.end()) {
    ss << *iter;
    ++iter;
    if (iter != paths.end()) {
      ss << ", ";
    }
  }

  ss << "]";
  return ss.str();
}

using ianvs::detail::StringTransform;

std::optional<std::regex> get_topic_filter(const std::vector<std::string>& filters) {
  std::stringstream ss;
  auto iter = filters.begin();
  while (iter != filters.end()) {
    ss << *iter;
    ++iter;
    if (iter != filters.end()) {
      ss << "|";
    }
  }

  if (ss.str().empty()) {
    return std::nullopt;
  }

  return std::regex(ss.str());
}

void merge_bags(const std::vector<std::filesystem::path>& inputs,
                const std::filesystem::path& output_path,
                const std::vector<std::string>& topics,
                const std::vector<std::string>& to_exclude) {
  rclcpp::get_logger("rosbag2_storage").set_level(rclcpp::Logger::Level::Warn);
  std::cout << "Merging " << print_bag_list(inputs) << " -> " << output_path << std::endl;

  std::vector<std::string> filters;
  std::vector<StringTransform> transforms;
  for (const auto& topic : topics) {
    const auto pos = topic.find(":");
    if (pos == std::string::npos) {
      // no need to parse filter
      filters.push_back(topic);
      continue;
    }

    filters.push_back(topic.substr(0, pos));
    transforms.push_back(StringTransform::from_arg(StringTransform::Type::Substitute, topic));
  }

  const auto topic_filter = get_topic_filter(filters);
  const auto exclude_filter = get_topic_filter(to_exclude);

  rosbag2_storage::StorageOptions output_options;
  output_options.uri = output_path;
  rosbag2_cpp::writers::SequentialWriter writer;
  writer.open(output_options, rosbag2_cpp::ConverterOptions{});

  ReaderVec readers;
  for (const auto& input : inputs) {
    if (!std::filesystem::exists(input)) {
      std::cout << "Error: invalid bag '" << input << "'" << std::endl;
      continue;
    }

    readers.emplace_back(input);
  }

  std::smatch match;
  std::set<std::string> seen;
  MsgVec msgs(readers.size());
  fill_messages(readers, msgs);
  MessageInfo to_write = get_next_message(msgs);
  while (to_write) {
    if (topic_filter && !std::regex_match(to_write.msg->topic_name, match, *topic_filter)) {
      fill_messages(readers, msgs);
      to_write = get_next_message(msgs);
      continue;
    }

    if (exclude_filter && std::regex_match(to_write.msg->topic_name, match, *exclude_filter)) {
      fill_messages(readers, msgs);
      to_write = get_next_message(msgs);
      continue;
    }

    for (const auto& transform : transforms) {
      to_write.msg->topic_name = transform.apply(to_write.msg->topic_name);
    }

    if (!seen.count(to_write.msg->topic_name)) {
      auto metadata = *to_write.metadata;
      metadata.name = to_write.msg->topic_name;
      writer.create_topic(metadata);
      seen.insert(to_write.msg->topic_name);
    }

    writer.write(to_write.msg);
    fill_messages(readers, msgs);
    to_write = get_next_message(msgs);
  }
}

std::filesystem::path normalize_bag_path(const std::filesystem::path& bag_path) {
  auto normed_path = std::filesystem::canonical(bag_path);
  if (std::filesystem::is_directory(normed_path)) {
    return normed_path;
  } else {
    return normed_path.parent_path();
  }
}

std::filesystem::path get_temp_output(const std::filesystem::path& last_bag) {
  std::filesystem::path output = last_bag.parent_path();
  const std::string output_name = "." + last_bag.stem().string();
  output /= output_name;
  return output;
}

int main(int argc, char** argv) {
  CLI::App app("Utility to merge multiple rosbags");
  argv = app.ensure_utf8(argv);
  app.allow_extras();

  std::filesystem::path output;
  std::vector<std::string> topics;
  std::vector<std::string> exclude;
  std::vector<std::filesystem::path> bags;
  app.add_option("bags", bags)->check(CLI::ExistingPath)->description("bags to take topics from");
  app.add_option("-o,--output", output)->description("optional output bag");
  app.add_option("-t,--topics", topics)->description("topics to include");
  app.add_option("-e,--exclude", exclude)->description("topics to exclude");

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    return app.exit(e);
  }

  if (bags.empty() || (bags.size() == 1 && topics.empty() && exclude.empty())) {
    return 0;  // no need to do any work
  }

  for (auto& path : bags) {
    path = normalize_bag_path(path);
  }

  // if the output is empty, we're merging to the last path in the input bags
  bool cleanup = false;
  auto& to_bag = bags.back();
  if (output.empty()) {
    cleanup = true;
    output = to_bag;
    to_bag = get_temp_output(to_bag);
    std::cout << "Moving last bag " << output << " -> " << to_bag << std::endl;
    std::filesystem::rename(output, to_bag);
  }

  merge_bags(bags, output, topics, exclude);
  if (cleanup) {
    std::filesystem::remove_all(to_bag);
  }

  return 0;
}
