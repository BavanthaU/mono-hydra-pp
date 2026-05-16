#include "ianvs/bag_reader.h"

#include <rosbag2_transport/reader_writer_factory.hpp>

namespace ianvs {

BagReader::BagReader(const std::filesystem::path& bagpath) {
  rosbag2_storage::StorageOptions opts;
  opts.uri = bagpath;
  reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(opts);
  if (!reader_) {
    return;
  }

  reader_->open(opts);

  const auto metadata = reader_->get_all_topics_and_types();
  for (const auto& data : metadata) {
    lookup_[data.name] = data;
  }
}

BagMessage::Ptr BagReader::next() const {
  while (reader_->has_next()) {
    auto msg = reader_->read_next();
    if (!msg) {
      continue;
    }

    auto iter = lookup_.find(msg->topic_name);
    if (iter == lookup_.end()) {
      continue;
    }

    return std::make_shared<BagMessage>(msg, iter->second);
  }

  return nullptr;
}

size_t BagReader::message_count() const {
  size_t num_messages = 0;
  for (const auto& entry : reader_->get_metadata().topics_with_message_count) {
    num_messages += entry.message_count;
  }

  return num_messages;
}

}  // namespace ianvs
