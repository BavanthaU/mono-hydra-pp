#include <gtest/gtest.h>
#include <ianvs/detail/bag_topic_checker.h>

namespace ianvs {

TEST(BagTopicChecker, EmptyFilterCorrect) {
  rosbag2_transport::PlayOptions options;
  BagTopicChecker checker(options);
  EXPECT_TRUE(checker.passes("/tf"));
  EXPECT_TRUE(checker.passes("/tf_static"));
}

TEST(BagTopicChecker, IncludeFilterCorrect) {
  rosbag2_transport::PlayOptions options;
  options.topics_to_filter = {"/tf", "/my_random_topic"};
  BagTopicChecker checker(options);
  EXPECT_TRUE(checker.passes("/tf"));
  EXPECT_FALSE(checker.passes("/tf_static"));
}

TEST(BagTopicChecker, IncludeRegexCorrect) {
  rosbag2_transport::PlayOptions options;
  options.regex_to_filter = ".*tf";
  BagTopicChecker checker(options);
  EXPECT_TRUE(checker.passes("/tf"));
  EXPECT_FALSE(checker.passes("/tf_static"));
}

TEST(BagTopicChecker, ExcludeFilterCorrect) {
  rosbag2_transport::PlayOptions options;
  options.exclude_topics_to_filter = {"/tf", "/my_random_topic"};
  BagTopicChecker checker(options);
  EXPECT_FALSE(checker.passes("/tf"));
  EXPECT_TRUE(checker.passes("/tf_static"));
}

TEST(BagTopicChecker, ExcludeRegexCorrect) {
  rosbag2_transport::PlayOptions options;
  options.exclude_regex_to_filter = ".*tf";
  BagTopicChecker checker(options);
  EXPECT_FALSE(checker.passes("/tf"));
  EXPECT_TRUE(checker.passes("/tf_static"));
}

TEST(BagTopicChecker, AllCorrect) {
  rosbag2_transport::PlayOptions options;
  options.topics_to_filter = {"/my_random_topic", "/bar", "/some_static_topic"};
  options.regex_to_filter = ".*tf";
  options.exclude_topics_to_filter = {"/tf_static", "/other"};
  options.exclude_regex_to_filter = ".*static.*";

  BagTopicChecker checker(options);
  EXPECT_TRUE(checker.passes("/tf"));
  EXPECT_FALSE(checker.passes("/tf_static"));
  EXPECT_TRUE(checker.passes("/my_random_topic"));
  EXPECT_FALSE(checker.passes("/some_static_topic"));
}

}  // namespace ianvs
