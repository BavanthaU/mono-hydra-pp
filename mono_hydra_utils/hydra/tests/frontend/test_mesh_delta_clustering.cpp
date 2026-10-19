#include <gtest/gtest.h>
#include <hydra/frontend/mesh_delta_clustering.h>
#include <kimera_pgmo/mesh_delta.h>

#include <algorithm>

namespace hydra::clustering {

TEST(MeshDeltaClustering, PreservesConnectedComponentsAndOriginalIndices) {
  kimera_pgmo::MeshDelta delta({0, 0, 0});
  const std::vector<float> positions{0.0f, 0.0f, 0.09f, 0.18f, 1.0f, 1.08f, 5.0f};
  for (float x : positions) {
    delta.addVertex({x, 0.0f, 0.0f}, {});
  }
  ClusteringConfig config;
  config.cluster_tolerance = 0.1;
  config.min_cluster_size = 2;
  config.max_cluster_size = 10;
  // Include coincident points, a transitive chain, shuffled input and an outlier.
  auto clusters = findClusters(config, delta, {5, 2, 0, 6, 4, 3, 1});
  for (auto& cluster : clusters) {
    std::sort(cluster.begin(), cluster.end());
  }
  std::sort(clusters.begin(), clusters.end());
  EXPECT_EQ(clusters, (Clusters{{0, 1, 2, 3}, {4, 5}}));

  config.max_cluster_size = 3;
  clusters = findClusters(config, delta, {0, 1, 2, 3, 4, 5, 6});
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_EQ(clusters.front(), (std::vector<size_t>{4, 5}));
}

}  // namespace hydra::clustering
