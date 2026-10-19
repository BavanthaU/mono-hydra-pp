#include <gtest/gtest.h>
#include <hydra/reconstruction/projective_integrator.h>

namespace hydra {
namespace {
using Measurement = ProjectiveIntegrator::VoxelMeasurement;

TEST(ProjectiveIntegrator, RobustWeightPreservesDiscoveryAndDefault) {
  VolumetricMap::Config map;
  map.truncation_distance = 0.45f;
  Measurement m;
  m.range = 2.0f;
  m.sdf = -0.3f;
  m.weight = 4.0f;
  TsdfVoxel voxel;
  voxel.distance = 0.0f;
  voxel.weight = 10.0f;
  ProjectiveIntegrator::Config config;
  config.num_threads = 1;
  EXPECT_FLOAT_EQ(ProjectiveIntegrator(config).computeRobustWeight(map, m, voxel), 4.0f);
  config.use_robust_weight = true;
  ProjectiveIntegrator integrator(config);
  voxel.weight = 0.0f;
  EXPECT_FLOAT_EQ(integrator.computeRobustWeight(map, m, voxel), 4.0f);
  voxel.weight = 10.0f;
  voxel.distance = map.truncation_distance;
  EXPECT_FLOAT_EQ(integrator.computeRobustWeight(map, m, voxel), 4.0f);
  voxel.distance = -map.truncation_distance;
  EXPECT_FLOAT_EQ(integrator.computeRobustWeight(map, m, voxel), 4.0f);
  voxel.distance = std::nextafter(map.truncation_distance, 0.0f);
  EXPECT_FLOAT_EQ(integrator.computeRobustWeight(map, m, voxel), 4.0f);
}

TEST(ProjectiveIntegrator, RobustInfluenceIsBoundedAndRangeAware) {
  VolumetricMap::Config map;
  ProjectiveIntegrator::Config config;
  config.num_threads = 1;
  config.use_robust_weight = true;
  ProjectiveIntegrator integrator(config);
  TsdfVoxel voxel;
  voxel.distance = 0.0f;
  voxel.weight = 5.0f;
  Measurement m;
  m.range = 2.0f;
  m.weight = 1.0f;
  EXPECT_FLOAT_EQ(integrator.computeRobustWeight(map, m, voxel), 1.0f);
  m.sdf = 0.25f;
  const auto near_weight = integrator.computeRobustWeight(map, m, voxel);
  EXPECT_LT(near_weight, 0.2f);
  m.range = 6.0f;
  EXPECT_GT(integrator.computeRobustWeight(map, m, voxel), near_weight);
  m.sdf = 100.0f;
  EXPECT_FLOAT_EQ(integrator.computeRobustWeight(map, m, voxel),
                  config.robust_weight_min_factor);
}

TEST(ProjectiveIntegrator, RobustFusionLimitsOutlierButAdaptsToPersistentChange) {
  VolumetricMap::Config map;
  ProjectiveIntegrator::Config config;
  config.num_threads = 1;
  config.use_robust_weight = true;
  ProjectiveIntegrator integrator(config);
  InputData data(nullptr);  // updateVoxel does not require a sensor
  data.timestamp_ns = 1;
  TsdfVoxel voxel;
  voxel.weight = 5.0f;
  voxel.distance = 0.0f;
  VoxelTuple tuple;
  tuple.tsdf = &voxel;
  Measurement m;
  m.weight = 1.0f;
  m.range = 2.0f;
  m.sdf = 0.25f;
  integrator.updateVoxel(map, data, m, tuple);
  EXPECT_LT(voxel.distance, 0.01f);  // ordinary average would move 4.17 cm
  for (int i = 0; i < 1000; ++i) {
    integrator.updateVoxel(map, data, m, tuple);
  }
  EXPECT_GT(voxel.distance, 0.24f);
}

TEST(ProjectiveIntegrator, ZeroWeightDoesNotPoisonUnobservedVoxel) {
  ProjectiveIntegrator::Config config;
  config.num_threads = 1;
  ProjectiveIntegrator integrator(config);
  TsdfVoxel voxel;
  VoxelTuple tuple;
  tuple.tsdf = &voxel;
  InputData data(nullptr);
  Measurement m;
  integrator.updateVoxel(VolumetricMap::Config{}, data, m, tuple);
  EXPECT_TRUE(std::isfinite(voxel.distance));
  EXPECT_FLOAT_EQ(voxel.weight, 0.0f);
}
}  // namespace
}  // namespace hydra
