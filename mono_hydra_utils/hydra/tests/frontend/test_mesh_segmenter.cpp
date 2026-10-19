/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * all rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <hydra/frontend/mesh_segmenter.h>
#include <hydra/input/camera.h>
#include <hydra/utils/pgmo_mesh_traits.h>
#include <kimera_pgmo/mesh_delta.h>

namespace hydra {

using kimera_pgmo::MeshDelta;

namespace {

struct NodeResult {
  uint32_t label;
  std::list<size_t> mesh_connections;
  BoundingBox bbox;
  bool is_active = true;
};

void checkBoundingBox(const BoundingBox& expected,
                      const BoundingBox& result,
                      double tolerance = 1.0e-6f) {
  EXPECT_NEAR(expected.dimensions.x(), result.dimensions.x(), tolerance);
  EXPECT_NEAR(expected.dimensions.y(), result.dimensions.y(), tolerance);
  EXPECT_NEAR(expected.dimensions.z(), result.dimensions.z(), tolerance);
  EXPECT_NEAR(expected.world_P_center.x(), result.world_P_center.x(), tolerance);
  EXPECT_NEAR(expected.world_P_center.y(), result.world_P_center.y(), tolerance);
  EXPECT_NEAR(expected.world_P_center.z(), result.world_P_center.z(), tolerance);
}

bool checkNode(const DynamicSceneGraph& graph,
               NodeId node_id,
               const NodeResult& expected) {
  const auto node = graph.findNode(node_id);
  if (!node) {
    return false;
  }

  auto& result = node->attributes<ObjectNodeAttributes>();
  EXPECT_EQ(expected.label, result.semantic_label);
  EXPECT_EQ(expected.mesh_connections, result.mesh_connections);
  checkBoundingBox(expected.bbox, result.bounding_box);
  EXPECT_EQ(expected.is_active, result.is_active);
  return true;
}

void stepSegmenter(const MeshDelta& delta,
                   kimera_pgmo::MeshOffsetInfo& offsets,
                   MeshSegmenter& segmenter,
                   DynamicSceneGraph& graph) {
  delta.updateMesh(*graph.mesh(), offsets);
  const auto clusters = segmenter.detect(0, delta, offsets);
  segmenter.updateGraph(0, offsets, clusters, graph);
}

void addPoints(MeshDelta& delta,
               uint32_t label,
               const Eigen::Vector3f& offset,
               const Eigen::Vector3f& scale,
               bool archive = false) {
  const auto corners = BoundingBox(scale, offset).corners();
  for (const auto& corner : corners) {
    kimera_pgmo::traits::VertexTraits traits;
    traits.properties.has_label = true;
    traits.label = label;
    delta.addVertex(corner, traits, archive);
  }
}

}  // namespace

TEST(MeshSegmenter, TestClustering) {
  MeshSegmenter::Config config;
  config.clustering.min_cluster_size = 4;
  MeshSegmenter segmenter(config, {1, 2});

  MeshDelta delta({0, 0, 0});
  addPoints(delta, 1, {1, 2, 3}, Eigen::Vector3f::Constant(0.1));
  addPoints(delta, 2, {4, 5, 6}, Eigen::Vector3f::Constant(0.1));

  const auto clusters = segmenter.detect(0, delta, {0, 0, 0});
  ASSERT_EQ(clusters.size(), 2u);
}

TEST(MeshSegmenter, TestIndicesRemapping) {
  Eigen::Vector3f dims = Eigen::Vector3f::Constant(0.1);
  const BoundingBox b1(dims, Eigen::Vector3f(1, 2, 3));
  const BoundingBox b2(dims, Eigen::Vector3f(4, 5, 6));

  MeshSegmenter::Config config;
  config.clustering.min_cluster_size = 4;
  MeshSegmenter segmenter(config, {1, 2});

  DynamicSceneGraph graph;
  kimera_pgmo::MeshOffsetInfo offsets;
  graph.setMesh(std::make_shared<spark_dsg::Mesh>());

  {  // original objects
    MeshDelta delta({0, 0, 0});
    addPoints(delta, 1, {1, 2, 3}, dims);
    addPoints(delta, 2, {4, 5, 6}, dims);
    stepSegmenter(delta, offsets, segmenter, graph);

    std::unordered_set<NodeId> active{"O0"_id, "O1"_id};
    EXPECT_EQ(active, segmenter.getActiveNodes());

    const std::map<NodeId, NodeResult> expected_nodes{
        {"O0"_id, {1, {0, 1, 2, 3, 4, 5, 6, 7}, b1}},
        {"O1"_id, {2, {8, 9, 10, 11, 12, 13, 14, 15}, b2}},
    };

    for (const auto& [node_id, expected] : expected_nodes) {
      SCOPED_TRACE("Object " + NodeSymbol(node_id).str());
      EXPECT_TRUE(checkNode(graph, node_id, expected));
    }
  }

  {  // swapped objects
    std::map<size_t, size_t> remap;
    for (size_t i = 0; i < 8; ++i) {
      // two corner sets are swapped in order
      remap[i] = i + 8;
      remap[i + 8] = i;
    }

    const auto tracking = MeshDelta::TrackingInfo::with_remap(0, 16, 0, remap);
    MeshDelta delta(tracking);
    addPoints(delta, 2, {4, 5, 6}, Eigen::Vector3f::Constant(0.1));
    addPoints(delta, 1, {1, 2, 3}, Eigen::Vector3f::Constant(0.1));
    stepSegmenter(delta, offsets, segmenter, graph);

    const std::map<NodeId, NodeResult> expected_nodes{
        {"O0"_id, {1, {8, 9, 10, 11, 12, 13, 14, 15}, b1}},
        {"O1"_id, {2, {0, 1, 2, 3, 4, 5, 6, 7}, b2}},
    };

    for (const auto& [node_id, expected] : expected_nodes) {
      SCOPED_TRACE("Object " + NodeSymbol(node_id).str());
      EXPECT_TRUE(checkNode(graph, node_id, expected));
    }
  }
}

TEST(MeshSegmenter, TestDeletedObject) {
  Eigen::Vector3f dims = Eigen::Vector3f::Constant(0.1);
  const spark_dsg::BoundingBox b2(dims, Eigen::Vector3f(4, 5, 6));

  MeshSegmenter::Config config;
  config.clustering.min_cluster_size = 4;
  MeshSegmenter segmenter(config, {1, 2});

  DynamicSceneGraph graph;
  kimera_pgmo::MeshOffsetInfo offsets;
  graph.setMesh(std::make_shared<spark_dsg::Mesh>());

  {  // setup original objects
    MeshDelta delta({0, 0, 0});
    addPoints(delta, 1, {1, 2, 3}, dims);
    addPoints(delta, 2, {4, 5, 6}, dims);
    stepSegmenter(delta, offsets, segmenter, graph);
  }

  {  // delete object one
    std::map<size_t, size_t> remap;
    for (size_t i = 0; i < 8; ++i) {
      // previous object gets moved up
      remap[i + 8] = i;
    }

    const auto tracking = MeshDelta::TrackingInfo::with_remap(0, 16, 0, remap);
    MeshDelta delta(tracking);
    addPoints(delta, 2, {4, 5, 6}, dims);
    stepSegmenter(delta, offsets, segmenter, graph);

    const std::map<NodeId, NodeResult> expected{
        {"O1"_id, {2, {0, 1, 2, 3, 4, 5, 6, 7}, b2}}};
    for (const auto& [node_id, node] : expected) {
      SCOPED_TRACE("Object " + NodeSymbol(node_id).str());
      EXPECT_TRUE(checkNode(graph, node_id, node));
    }
  }
}

TEST(MeshSegmenter, TestArchivedObject) {
  Eigen::Vector3f dims = Eigen::Vector3f::Constant(0.1);
  const BoundingBox b1(dims, Eigen::Vector3f(1, 2, 3));
  const BoundingBox b2(dims, Eigen::Vector3f(4, 5, 6));

  MeshSegmenter::Config config;
  config.clustering.min_cluster_size = 4;
  MeshSegmenter segmenter(config, {1, 2});

  DynamicSceneGraph graph;
  kimera_pgmo::MeshOffsetInfo offsets;
  graph.setMesh(std::make_shared<spark_dsg::Mesh>());

  {  // setup original objects
    MeshDelta delta({0, 0, 0});
    addPoints(delta, 1, {1, 2, 3}, dims);
    addPoints(delta, 2, {4, 5, 6}, dims);
    stepSegmenter(delta, offsets, segmenter, graph);
  }

  {  // archive object two
    std::map<size_t, size_t> remap;
    for (size_t i = 0; i < 8; ++i) {
      // archived object gets moved down
      remap[i + 8] = i;
      // active object gets moved up
      remap[i] = i + 8;
    }

    const auto tracking = MeshDelta::TrackingInfo::with_remap(0, 16, 0, remap);
    MeshDelta delta(tracking);
    addPoints(delta, 2, {4, 5, 6}, dims, true);
    addPoints(delta, 1, {1, 2, 3}, dims);
    addPoints(delta, 2, {4, 5, 6}, dims);
    stepSegmenter(delta, offsets, segmenter, graph);

    const std::map<NodeId, NodeResult> expected_nodes{
        {"O0"_id, {1, {8, 9, 10, 11, 12, 13, 14, 15}, b1}},
        {"O1"_id, {2, {0, 1, 2, 3, 4, 5, 6, 7}, b2, false}},
        {"O2"_id, {2, {16, 17, 18, 19, 20, 21, 22, 23}, b2}},
    };
    for (const auto& [node_id, expected] : expected_nodes) {
      SCOPED_TRACE("Object " + NodeSymbol(node_id).str());
      EXPECT_TRUE(checkNode(graph, node_id, expected));
    }
  }
}

TEST(MeshSegmenter, TestDeltaWithOffset) {
  Eigen::Vector3f dims = Eigen::Vector3f::Constant(0.1);
  const BoundingBox b1(dims, Eigen::Vector3f(1, 2, 3));

  MeshSegmenter::Config config;
  config.clustering.min_cluster_size = 4;
  MeshSegmenter segmenter(config, {1, 2});

  DynamicSceneGraph graph;
  kimera_pgmo::MeshOffsetInfo offsets;
  graph.setMesh(std::make_shared<spark_dsg::Mesh>());

  {  // add 8 archived vertices to mesh
    MeshDelta delta({0, 0, 0});
    addPoints(delta, 1, {1, 2, 3}, dims, true);
    delta.updateMesh(*graph.mesh(), offsets);
  }

  {  // add actual object
    MeshDelta delta({0, 0, 0});
    addPoints(delta, 1, {1, 2, 3}, dims);
    stepSegmenter(delta, offsets, segmenter, graph);

    const std::map<NodeId, NodeResult> expected_nodes{
        {"O0"_id, {1, {8, 9, 10, 11, 12, 13, 14, 15}, b1}},
    };
    for (const auto& [node_id, expected] : expected_nodes) {
      SCOPED_TRACE("Object " + NodeSymbol(node_id).str());
      EXPECT_TRUE(checkNode(graph, node_id, expected));
    }
  }
}

TEST(MeshSegmenter, RemappedMovingSurfaceKeepsObjectIdentity) {
  MeshSegmenter::Config config;
  config.clustering.min_cluster_size = 4;
  MeshSegmenter segmenter(config, {4});
  DynamicSceneGraph graph;
  graph.setMesh(std::make_shared<spark_dsg::Mesh>());
  kimera_pgmo::MeshOffsetInfo offsets;
  const Eigen::Vector3f dimensions(0.1f, 0.02f, 0.1f);

  MeshDelta first({0, 0, 0});
  addPoints(first, 4, {1, 2, 3}, dimensions);
  stepSegmenter(first, offsets, segmenter, graph);
  ASSERT_EQ(segmenter.getActiveNodes().size(), 1u);

  std::map<size_t, size_t> remap;
  for (size_t i = 0; i < 8; ++i) {
    remap[i] = i;
  }
  // TSDF updates move the same tracked vertices beyond the old thin bounding box.
  // Association must use their current geometry, not instantiate another object.
  MeshDelta second(MeshDelta::TrackingInfo::with_remap(0, 8, 0, remap));
  addPoints(second, 4, {1, 2.3f, 3}, dimensions);
  stepSegmenter(second, offsets, segmenter, graph);
  EXPECT_EQ(segmenter.getActiveNodes(), (std::unordered_set<NodeId>{"O0"_id}));
  ASSERT_TRUE(graph.hasNode("O0"_id));
  EXPECT_NEAR(graph.getNode("O0"_id).attributes().position.y(), 2.3, 1.0e-5);
}

TEST(MeshSegmenter, ProjectedOverlapRequiresRepeatedVisibleEvidence) {
  MeshSegmenter::Config config;
  config.clustering.min_cluster_size = 4;
  config.use_temporal_overlap = true;
  MeshSegmenter segmenter(config, {4});
  DynamicSceneGraph graph;
  graph.setMesh(std::make_shared<spark_dsg::Mesh>());
  kimera_pgmo::MeshOffsetInfo offsets;
  MeshDelta delta({0, 0, 0});
  addPoints(delta, 4, {0, 0, 3}, {0.6f, 0.8f, 0.02f});
  addPoints(delta, 4, {0, 0, 3.12f}, {0.6f, 0.8f, 0.02f});
  delta.updateMesh(*graph.mesh(), offsets);
  MeshSegmenter::LabelClusters clusters;
  clusters[4] = {{{0, 0, 3}, {0, 1, 2, 3, 4, 5, 6, 7}},
                  {{0, 0, 3.12}, {8, 9, 10, 11, 12, 13, 14, 15}}};
  Camera::Config k;
  k.width = 160; k.height = 120; k.fx = k.fy = 100;
  k.cx = 80; k.cy = 60; k.min_range = 0.1; k.max_range = 10;
  k.extrinsics = ParamSensorExtrinsics::Config{};
  InputData observation(std::make_shared<Camera>(k, "camera"));
  observation.world_T_body = Eigen::Isometry3d::Identity();
  observation.depth_image = cv::Mat(120, 160, CV_32FC1, cv::Scalar(3.06f));
  segmenter.updateGraph(1000000000, offsets, clusters, graph, &observation);
  std::map<size_t, size_t> identity;
  for (size_t i = 0; i < 16; ++i) { identity[i] = i; }
  MeshDelta repeated(MeshDelta::TrackingInfo::with_remap(0, 16, 0, identity));
  addPoints(repeated, 4, {0, 0, 3}, {0.6f, 0.8f, 0.02f});
  addPoints(repeated, 4, {0, 0, 3.12f}, {0.6f, 0.8f, 0.02f});
  repeated.updateMesh(*graph.mesh(), offsets);
  ASSERT_EQ(segmenter.getActiveNodes().size(), 2u);
  segmenter.updateGraph(1100000000, offsets, {}, graph, &observation);
  EXPECT_EQ(segmenter.getActiveNodes().size(), 2u);
  segmenter.updateGraph(1200000000, offsets, {}, graph, &observation);
  EXPECT_EQ(segmenter.getActiveNodes().size(), 1u);
  EXPECT_EQ(graph.getNode("O0"_id).attributes<ObjectNodeAttributes>().mesh_connections.size(), 16u);
}

TEST(MeshSegmenter, TemporalOverlapDoesNotMergeOccludedObjects) {
  MeshSegmenter::Config config;
  config.clustering.min_cluster_size = 4;
  config.use_temporal_overlap = true;
  MeshSegmenter segmenter(config, {4});
  DynamicSceneGraph graph;
  graph.setMesh(std::make_shared<spark_dsg::Mesh>());
  kimera_pgmo::MeshOffsetInfo offsets;
  MeshDelta delta({0, 0, 0});
  addPoints(delta, 4, {0, 0, 3}, {0.6f, 0.8f, 0.02f});
  addPoints(delta, 4, {0, 0, 3.12f}, {0.6f, 0.8f, 0.02f});
  delta.updateMesh(*graph.mesh(), offsets);
  MeshSegmenter::LabelClusters clusters;
  clusters[4] = {{{0, 0, 3}, {0, 1, 2, 3, 4, 5, 6, 7}},
                  {{0, 0, 3.12}, {8, 9, 10, 11, 12, 13, 14, 15}}};
  Camera::Config k;
  k.width = 160; k.height = 120; k.fx = k.fy = 100;
  k.cx = 80; k.cy = 60; k.min_range = 0.1; k.max_range = 10;
  k.extrinsics = ParamSensorExtrinsics::Config{};
  InputData observation(std::make_shared<Camera>(k, "camera"));
  observation.world_T_body = Eigen::Isometry3d::Identity();
  observation.depth_image = cv::Mat(120, 160, CV_32FC1, cv::Scalar(1.0f));
  segmenter.updateGraph(1000000000, offsets, clusters, graph, &observation);
  std::map<size_t, size_t> identity;
  for (size_t i = 0; i < 16; ++i) { identity[i] = i; }
  MeshDelta repeated(MeshDelta::TrackingInfo::with_remap(0, 16, 0, identity));
  addPoints(repeated, 4, {0, 0, 3}, {0.6f, 0.8f, 0.02f});
  addPoints(repeated, 4, {0, 0, 3.12f}, {0.6f, 0.8f, 0.02f});
  repeated.updateMesh(*graph.mesh(), offsets);
  for (uint64_t t = 1100000000; t <= 1800000000; t += 100000000) {
    segmenter.updateGraph(t, offsets, {}, graph, &observation);
  }
  EXPECT_EQ(segmenter.getActiveNodes().size(), 2u);
}

}  // namespace hydra
