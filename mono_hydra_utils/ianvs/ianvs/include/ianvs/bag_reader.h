#pragma once
#include <filesystem>

#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>

namespace ianvs {

struct BagMessage {
  using Ptr = std::shared_ptr<BagMessage>;
  const std::shared_ptr<rosbag2_storage::SerializedBagMessage> contents;
  const rosbag2_storage::TopicMetadata metadata;

  BagMessage(const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& contents,
             const rosbag2_storage::TopicMetadata& metadata)
      : contents(contents), metadata(metadata) {
    if (!contents) {
      throw std::runtime_error("invalid message!");
    }
  }

  std::string topic() const { return contents->topic_name; }
  std::string type() const { return metadata.type; }
  rclcpp::SerializedMessage serialized() const {
    return rclcpp::SerializedMessage(*contents->serialized_data);
  }

  template <typename T>
  bool is() const {
    return metadata.type == rosidl_generator_traits::name<T>();
  }

  template <typename T>
  typename T::Ptr as() const {
    const auto name = rosidl_generator_traits::name<T>();
    if (metadata.type != name) {
      return nullptr;
    }

    auto msg = std::make_shared<T>();
    const rclcpp::Serialization<T> serialization;
    const auto serialized_contents = serialized();
    serialization.deserialize_message(&serialized_contents, msg.get());
    return msg;
  }
};

class BagReader {
 public:
  explicit BagReader(const std::filesystem::path& bagpath);
  BagMessage::Ptr next() const;
  size_t message_count() const;
  operator bool() const { return reader_ != nullptr; }

 private:
  std::unique_ptr<rosbag2_cpp::Reader> reader_;
  std::map<std::string, rosbag2_storage::TopicMetadata> lookup_;
};

}  // namespace ianvs
