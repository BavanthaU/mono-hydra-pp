/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
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
#pragma once
#include "hydra/common/dsg_types.h"
#include "hydra/common/output_sink.h"
#include "hydra/frontend/mesh_delta_clustering.h"
#include "hydra/input/input_data.h"

namespace kimera_pgmo {
class MeshDelta;
struct MeshOffsetInfo;
}  // namespace kimera_pgmo

namespace hydra {

using clustering::Clusters;
using clustering::LabelIndices;

class MeshSegmenter {
 public:
  struct Cluster {
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    std::vector<size_t> indices;
  };

  using LabelClusters = std::map<uint32_t, std::vector<Cluster>>;
  using Sink = OutputSink<uint64_t,
                          const kimera_pgmo::MeshDelta&,
                          const LabelIndices&,
                          const LabelClusters&>;

  struct Config {
    std::string layer_id = DsgLayers::OBJECTS;
    clustering::ClusteringConfig clustering;
    BoundingBox::Type bounding_box_type = BoundingBox::Type::AABB;
    std::string timer_namespace = "frontend/objects";
    std::vector<Sink::Factory> sinks;
    bool use_temporal_overlap = false;
    double association_max_gap_m = 0.2;
    double association_depth_tolerance_m = 0.2;
    double association_min_overlap = 0.6;
    double association_max_age_s = 2.0;
    int association_confirmations = 3;
  } const config;

  explicit MeshSegmenter(const Config& config, const std::set<uint32_t>& labels);

  LabelClusters detect(uint64_t timestamp_ns,
                       const kimera_pgmo::MeshDelta& active,
                       const kimera_pgmo::MeshOffsetInfo& offsets);

  void updateGraph(uint64_t timestamp,
                   const kimera_pgmo::MeshOffsetInfo& offsets,
                   const LabelClusters& clusters,
                   DynamicSceneGraph& graph,
                   const InputData* observation = nullptr);

  std::unordered_set<NodeId> getActiveNodes() const;

 private:
  void updateOldNodes(const kimera_pgmo::MeshOffsetInfo& offsets,
                      DynamicSceneGraph& graph);

  void addNodeToGraph(DynamicSceneGraph& graph,
                      const Cluster& cluster,
                      uint32_t label,
                      uint64_t timestamp);

  void updateNodeInGraph(DynamicSceneGraph& graph,
                         const Cluster& cluster,
                         const SceneGraphNode& node,
                         uint64_t timestamp);

  void mergeActiveNodes(DynamicSceneGraph& graph, uint32_t label);
  void mergeTemporalNodes(DynamicSceneGraph& graph,
                          uint64_t timestamp,
                          const InputData& observation);

 private:
  NodeSymbol next_node_id_;
  std::set<uint32_t> labels_;
  std::map<uint32_t, std::set<NodeId>> active_nodes_;
  Sink::List sinks_;
  struct Evidence {
    uint64_t stamp = 0;
    int count = 0;
  };
  std::map<std::pair<NodeId, NodeId>, Evidence> association_evidence_;
  uint64_t last_association_stamp_ = 0;
};

void declare_config(MeshSegmenter::Config& config);

}  // namespace hydra
