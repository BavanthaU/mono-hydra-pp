#include <gtest/gtest.h>
#include <hydra/reconstruction/geometric_reliability.h>
#include <hydra/reconstruction/projective_integrator.h>

namespace hydra {
namespace {
std::shared_ptr<Camera> camera() {
  Camera::Config k;
  k.width = 80; k.height = 60; k.fx = k.fy = 60;
  k.cx = 40; k.cy = 30; k.min_range = 0.1; k.max_range = 10;
  k.extrinsics = ParamSensorExtrinsics::Config{};
  return std::make_shared<Camera>(k, "test");
}

TEST(GeometricReliability, BoundedConsistencyAndUnknownVisibility) {
  GeometricReliability::Config config;
  config.enabled = true;
  GeometricReliability score(config);
  InputData d(camera());
  d.world_T_body = Eigen::Isometry3d::Identity();
  d.depth_image = cv::Mat(60, 80, CV_32FC1, cv::Scalar(3.0f));
  d.timestamp_ns = 1000000000;
  EXPECT_FLOAT_EQ(score.compute(d).at<float>(30, 40), 1.0f);
  d.timestamp_ns += 100000000;
  d.world_T_body.translation().x() = 0.1;
  EXPECT_FLOAT_EQ(score.compute(d).at<float>(30, 40), 1.0f);
  d.timestamp_ns += 100000000;
  d.world_T_body.translation().x() = 0.2;
  d.depth_image.setTo(3.2f);
  const auto result = score.compute(d);
  EXPECT_GE(result.at<float>(30, 40), config.min_factor);
  EXPECT_LT(result.at<float>(30, 40), 0.5f);
  // A large occlusion/disocclusion is unknown, not evidence to suppress discovery.
  d.timestamp_ns += 100000000;
  d.world_T_body.translation().x() = 0.3;
  d.depth_image.setTo(1.0f);
  EXPECT_FLOAT_EQ(score.compute(d).at<float>(30, 40), 1.0f);
}

TEST(GeometricReliability, OpticalAxisTranslationUsesAxialDepthAndResets) {
  GeometricReliability::Config config; config.enabled = true;
  GeometricReliability score(config);
  InputData d(camera()); d.world_T_body = Eigen::Isometry3d::Identity();
  d.depth_image = cv::Mat(60, 80, CV_32FC1, cv::Scalar(3.0f));
  d.timestamp_ns = 1000000000;
  score.compute(d);
  d.timestamp_ns += 100000000;
  d.world_T_body.translation().z() = 0.1;
  d.depth_image.setTo(2.9f);
  EXPECT_NEAR(score.compute(d).at<float>(30, 40), 1.0f, 1e-5);
  d.timestamp_ns = 1;
  d.depth_image.setTo(3.1f);
  EXPECT_FLOAT_EQ(score.compute(d).at<float>(30, 40), 1.0f);
  EXPECT_TRUE(GeometricReliability({}).compute(d).empty());
}

TEST(GeometricReliability, WeightReachesVoxelAndDefaultIsUnchanged) {
  InputData d(camera()); d.world_T_body = Eigen::Isometry3d::Identity();
  d.depth_image = cv::Mat(60, 80, CV_32FC1, cv::Scalar(3.0f));
  ASSERT_TRUE(d.getSensor().finalizeRepresentations(d));
  ProjectiveIntegrator::Config c; c.num_threads = 1;
  ProjectiveIntegrator integrator(c);
  VolumetricMap::Config map;
  const Eigen::Vector3f point(0, 0, 3);
  const auto base = integrator.getVoxelMeasurement(map, d, {}, point);
  ASSERT_TRUE(base.valid);
  d.geometric_reliability = cv::Mat(60, 80, CV_32FC1, cv::Scalar(0.25f));
  const auto weighted = integrator.getVoxelMeasurement(map, d, {}, point);
  ASSERT_TRUE(weighted.valid);
  EXPECT_NEAR(weighted.weight, 0.25f * base.weight, 1e-7);
  EXPECT_FLOAT_EQ(weighted.sdf, base.sdf);
}
}  // namespace
}  // namespace hydra
