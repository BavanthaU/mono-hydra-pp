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
#include "hydra/frontend/mesh_segmenter.h"

#include <config_utilities/config.h>
#include <config_utilities/types/conversions.h>
#include <config_utilities/types/enum.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <glog/stl_logging.h>
#include <kimera_pgmo/mesh_delta.h>
#include <kimera_pgmo/mesh_traits.h>
#include <spark_dsg/bounding_box_extraction.h>
#include <spark_dsg/printing.h>

#include "hydra/utils/mesh_utilities.h"
#include "hydra/utils/timing_utilities.h"
#include "hydra/input/camera.h"

namespace hydra {

using Cluster = MeshSegmenter::Cluster;
using LabelClusters = MeshSegmenter::LabelClusters;
using timing::ScopedTimer;

void declare_config(MeshSegmenter::Config& config) {
  using namespace config;
  name("MeshSegmenterConfig");
  field(config.layer_id, "layer_id");
  field(config.clustering, "clustering", false);
  enum_field(config.bounding_box_type,
             "bounding_box_type",
             {{spark_dsg::BoundingBox::Type::INVALID, "INVALID"},
              {spark_dsg::BoundingBox::Type::AABB, "AABB"},
              {spark_dsg::BoundingBox::Type::OBB, "OBB"},
              {spark_dsg::BoundingBox::Type::RAABB, "RAABB"}});
  field(config.timer_namespace, "timer_namespace");
  field(config.sinks, "sinks");
  field(config.use_temporal_overlap, "use_temporal_overlap");
  field(config.association_max_gap_m, "association_max_gap_m");
  field(config.association_depth_tolerance_m, "association_depth_tolerance_m");
  field(config.association_min_overlap, "association_min_overlap");
  field(config.association_max_age_s, "association_max_age_s");
  field(config.association_confirmations, "association_confirmations");
  check(config.association_max_gap_m, GT, 0.0, "association_max_gap_m");
  check(config.association_depth_tolerance_m, GT, 0.0, "association_depth_tolerance_m");
  check(config.association_min_overlap, GT, 0.0, "association_min_overlap");
  check(config.association_min_overlap, LE, 1.0, "association_min_overlap");
  check(config.association_max_age_s, GT, 0.0, "association_max_age_s");
  check(config.association_confirmations, GE, 2, "association_confirmations");
}

template <typename LList, typename RList>
void mergeList(LList& lhs, const RList& rhs) {
  std::unordered_set<size_t> seen(lhs.begin(), lhs.end());
  for (const auto idx : rhs) {
    if (seen.count(idx)) {
      continue;
    }

    lhs.push_back(idx);
    seen.insert(idx);
  }
}

inline bool nodesMatch(const SceneGraphNode& lhs_node, const SceneGraphNode& rhs_node) {
  return lhs_node.attributes<SemanticNodeAttributes>().bounding_box.contains(
      rhs_node.attributes().position);
}

inline bool nodesMatch(const Cluster& cluster, const SceneGraphNode& node) {
  return node.attributes<SemanticNodeAttributes>().bounding_box.contains(
      cluster.centroid);
}

// TODO(nathan) move node ID to not be here
MeshSegmenter::MeshSegmenter(const Config& config, const std::set<uint32_t>& labels)
    : config(config::checkValid(config)),
      next_node_id_('O', 0),
      labels_(labels),
      sinks_(Sink::instantiate(config.sinks)) {
  VLOG(2) << "[Mesh Segmenter] using labels: " << clustering::printLabels(labels_);
  for (const auto& label : labels_) {
    active_nodes_[label] = std::set<NodeId>();
  }
}

LabelClusters MeshSegmenter::detect(uint64_t stamp_ns,
                                    const kimera_pgmo::MeshDelta& delta,
                                    const kimera_pgmo::MeshOffsetInfo& offsets) {
  ScopedTimer timer(config.timer_namespace + "_detection", stamp_ns, true, 1, false);

  LabelClusters label_clusters;
  if (!delta.getNumActiveVertices()) {
    VLOG(2) << "[Mesh Segmenter] No active indices in mesh";
    return label_clusters;
  }

  const auto label_indices = clustering::getLabelIndices(labels_, delta);
  if (label_indices.empty()) {
    VLOG(2) << "[Mesh Segmenter] No vertices found matching desired labels";
    Sink::callAll(sinks_, stamp_ns, delta, label_indices, {});
    return label_clusters;
  }

  for (const auto& [label, indices] : label_indices) {
    if (indices.size() < config.clustering.min_cluster_size) {
      continue;
    }

    const auto result = clustering::findClusters(config.clustering, delta, indices);

    auto iter = label_clusters.insert({label, {}}).first;
    auto& clusters = iter->second;
    for (const auto& cluster_indices : result) {
      auto& cluster = clusters.emplace_back();
      for (const auto local_idx : cluster_indices) {
        cluster.indices.push_back(local_idx);
        cluster.centroid += delta.getVertex(local_idx).pos.cast<double>();
      }

      if (cluster_indices.size()) {
        cluster.centroid /= cluster_indices.size();
      }
    }

    VLOG(2) << "[Mesh Segmenter] Found " << clusters.size() << " cluster(s) of label "
            << label;
  }

  Sink::callAll(sinks_, stamp_ns, delta, label_indices, label_clusters);

  // TODO(nathan) fix this
  for (auto& [label, clusters] : label_clusters) {
    for (auto& cluster : clusters) {
      for (auto& idx : cluster.indices) {
        idx = offsets.toGlobalVertex(idx);
      }
    }
  }

  return label_clusters;
}

void MeshSegmenter::updateOldNodes(const kimera_pgmo::MeshOffsetInfo& offsets,
                                   DynamicSceneGraph& graph) {
  for (auto& [label, label_nodes] : active_nodes_) {
    auto iter = label_nodes.begin();
    while (iter != label_nodes.end()) {
      const auto node_id = *iter;
      auto& attrs = graph.getNode(node_id).attributes<ObjectNodeAttributes>();

      // remap and prune mesh connections
      VLOG(20) << "Updating node " << NodeSymbol(node_id).str() << " with connections "
               << attrs.mesh_connections;
      kimera_pgmo::MeshOffsetInfo::RemapStats stats;
      offsets.remapVertexIndices(attrs.mesh_connections, &stats);
      VLOG(20) << "After update: " << attrs.mesh_connections << std::boolalpha
               << " (active: " << !stats.all_archived << ")";
      if (attrs.mesh_connections.size() < config.clustering.min_cluster_size) {
        graph.removeNode(node_id);
        iter = label_nodes.erase(iter);
        continue;
      }

      // TSDF updates move surviving vertices even when their IDs are unchanged.
      // Association below must use the remapped geometry, especially for thin objects.
      updateObjectGeometry(*graph.mesh(), attrs);
      attrs.is_active = !stats.all_archived;
      if (!attrs.is_active) {
        iter = label_nodes.erase(iter);
      } else {
        ++iter;
      }
    }
  }

  for (const auto& [label, label_nodes] : active_nodes_) {
    VLOG(10) << "Active nodes for label " << label << ": "
             << displayNodeSymbolContainer(label_nodes);
  }
}

void MeshSegmenter::updateGraph(uint64_t timestamp_ns,
                                const kimera_pgmo::MeshOffsetInfo& offsets,
                                const LabelClusters& clusters,
                                DynamicSceneGraph& graph,
                                const InputData* observation) {
  ScopedTimer timer(config.timer_namespace + "_graph_update", timestamp_ns);
  if (!graph.hasMesh()) {
    LOG(ERROR) << "Unable to update graph without mesh!";
    return;
  }
  updateOldNodes(offsets, graph);

  for (auto&& [label, clusters_for_label] : clusters) {
    for (const auto& cluster : clusters_for_label) {
      bool matches_prev_node = false;
      std::vector<NodeId> nodes_not_in_graph;
      for (const auto& prev_node_id : active_nodes_.at(label)) {
        const auto& prev_node = graph.getNode(prev_node_id);
        bool matches = nodesMatch(cluster, prev_node);
        if (config.use_temporal_overlap) {
          // Tracked mesh vertices are direct identity evidence. Geometric-only
          // associations below require repeated, visible projected support.
          const auto& connections = prev_node.attributes<ObjectNodeAttributes>().mesh_connections;
          const std::unordered_set<size_t> previous(connections.begin(), connections.end());
          size_t shared = 0;
          for (const auto index : cluster.indices) {
            shared += previous.count(index);
          }
          matches = shared > 0 && shared >= 0.5 * std::min(previous.size(), cluster.indices.size());
        }
        if (matches) {
          updateNodeInGraph(graph, cluster, prev_node, timestamp_ns);
          matches_prev_node = true;
          break;
        }
      }

      if (!matches_prev_node) {
        addNodeToGraph(graph, cluster, label, timestamp_ns);
      }

      if (!config.use_temporal_overlap) {
        mergeActiveNodes(graph, label);
      }
    }
  }
  if (config.use_temporal_overlap && observation) {
    mergeTemporalNodes(graph, timestamp_ns, *observation);
  }
}

void MeshSegmenter::mergeTemporalNodes(DynamicSceneGraph& graph,
                                        uint64_t timestamp,
                                        const InputData& observation) {
  if (timestamp <= last_association_stamp_) {
    association_evidence_.clear();
    last_association_stamp_ = timestamp;
    return;
  }
  last_association_stamp_ = timestamp;
  const auto* camera = dynamic_cast<const Camera*>(&observation.getSensor());
  if (!camera || observation.depth_image.type() != CV_32FC1 ||
      !observation.getSensorPose().matrix().allFinite()) {
    association_evidence_.clear();
    return;
  }
  const auto camera_T_world = observation.getSensorPose().inverse().cast<float>();
  const int width = observation.depth_image.cols, height = observation.depth_image.rows;
  const int columns = (width + 7) / 8;
  std::map<NodeId, std::set<int>> footprints;
  for (const auto& [label, ids] : active_nodes_) {
    for (const auto id : ids) {
      const auto& attrs = graph.getNode(id).attributes<ObjectNodeAttributes>();
      if (timestamp < attrs.last_update_time_ns ||
          (timestamp - attrs.last_update_time_ns) * 1.0e-9 > config.association_max_age_s) {
        continue;
      }
      auto& footprint = footprints[id];
      // Bound projection work per object; ordering is stable between updates.
      const size_t stride = std::max<size_t>(1, (attrs.mesh_connections.size() + 511) / 512);
      size_t i = 0;
      for (const auto index : attrs.mesh_connections) {
        if (i++ % stride || index >= graph.mesh()->numVertices()) {
          continue;
        }
        const Eigen::Vector3f p = camera_T_world * graph.mesh()->pos(index);
        int u, v;
        if (!p.allFinite() || p.z() <= 0 || !camera->projectPointToImagePlane(p, u, v) ||
            u < 0 || v < 0 || u >= width || v >= height) {
          continue;
        }
        const float z = observation.depth_image.at<float>(v, u);
        if (!std::isfinite(z) || z <= 0 ||
            std::abs(z - p.z()) > config.association_depth_tolerance_m) {
          continue;
        }
        // A small splat tolerates pixel discretization, without a full-image mask.
        for (int dv = -1; dv <= 1; ++dv) {
          for (int du = -1; du <= 1; ++du) {
            const int x = u / 8 + du, y = v / 8 + dv;
            if (x >= 0 && y >= 0 && x < columns && y < (height + 7) / 8) {
              footprint.insert(y * columns + x);
            }
          }
        }
      }
    }
  }
  std::map<std::pair<NodeId, NodeId>, Evidence> next;
  std::map<NodeId, size_t> candidate_count;
  for (const auto& [label, ids] : active_nodes_) {
    for (auto a = ids.begin(); a != ids.end(); ++a) {
      for (auto b = std::next(a); b != ids.end(); ++b) {
        const auto& x = footprints[*a];
        const auto& y = footprints[*b];
        if (x.size() < 6 || y.size() < 6) {
          continue;
        }
        const auto& lhs = graph.getNode(*a).attributes<ObjectNodeAttributes>().bounding_box;
        const auto& rhs = graph.getNode(*b).attributes<ObjectNodeAttributes>().bounding_box;
        // World AABBs from actual corners also support rotated bounding boxes.
        auto bounds = [](const BoundingBox& box) {
          Eigen::Vector3f lo = Eigen::Vector3f::Constant(std::numeric_limits<float>::infinity());
          Eigen::Vector3f hi = -lo;
          for (const auto& corner : box.corners()) {
            lo = lo.cwiseMin(corner); hi = hi.cwiseMax(corner);
          }
          return std::make_pair(lo, hi);
        };
        const auto l = bounds(lhs), r = bounds(rhs);
        const float gap = (l.first.cwiseMax(r.first) - l.second.cwiseMin(r.second))
                              .cwiseMax(0.0f).norm();
        if (gap > config.association_max_gap_m) {
          continue;
        }
        size_t overlap = 0;
        for (const int pixel : x) { overlap += y.count(pixel); }
        if (double(overlap) / std::min(x.size(), y.size()) < config.association_min_overlap) {
          continue;
        }
        const auto key = std::make_pair(*a, *b);
        Evidence evidence;
        const auto old = association_evidence_.find(key);
        if (old != association_evidence_.end() && timestamp > old->second.stamp &&
            (timestamp - old->second.stamp) * 1.0e-9 <= config.association_max_age_s) {
          evidence.count = old->second.count;
        }
        evidence.stamp = timestamp;
        ++evidence.count;
        next[key] = evidence;
        ++candidate_count[*a]; ++candidate_count[*b];
      }
    }
  }
  for (const auto& [pair, evidence] : next) {
    if (evidence.count < config.association_confirmations ||
        candidate_count[pair.first] != 1 || candidate_count[pair.second] != 1) {
      continue;
    }
    auto& lhs = graph.getNode(pair.first).attributes<ObjectNodeAttributes>();
    const auto& rhs = graph.getNode(pair.second).attributes<ObjectNodeAttributes>();
    mergeList(lhs.mesh_connections, rhs.mesh_connections);
    lhs.last_update_time_ns = std::max(lhs.last_update_time_ns, rhs.last_update_time_ns);
    active_nodes_.at(lhs.semantic_label).erase(pair.second);
    graph.removeNode(pair.second);
    updateObjectGeometry(*graph.mesh(), lhs);
    VLOG(2) << "TemporalObjectMerge stamp=" << timestamp << " keep=" << pair.first
            << " remove=" << pair.second << " confirmations=" << evidence.count;
  }
  association_evidence_.clear();
  for (const auto& [pair, evidence] : next) {
    if (graph.hasNode(pair.first) && graph.hasNode(pair.second)) {
      association_evidence_.insert({pair, evidence});
    }
  }
}

void MeshSegmenter::mergeActiveNodes(DynamicSceneGraph& graph, uint32_t label) {
  std::set<NodeId> merged_nodes;

  auto& curr_active = active_nodes_.at(label);
  for (const auto& node_id : curr_active) {
    if (merged_nodes.count(node_id)) {
      continue;
    }
    const auto& node = graph.getNode(node_id);

    std::list<NodeId> to_merge;
    for (const auto& other_id : curr_active) {
      if (node_id == other_id) {
        continue;
      }

      if (merged_nodes.count(other_id)) {
        continue;
      }

      const auto& other = graph.getNode(other_id);
      if (nodesMatch(node, other) || nodesMatch(other, node)) {
        to_merge.push_back(other_id);
      }
    }

    auto& attrs = node.attributes<ObjectNodeAttributes>();
    for (const auto& other_id : to_merge) {
      const auto& other = graph.getNode(other_id);
      auto& other_attrs = other.attributes<ObjectNodeAttributes>();
      mergeList(attrs.mesh_connections, other_attrs.mesh_connections);
      graph.removeNode(other_id);
      merged_nodes.insert(other_id);
    }

    if (!to_merge.empty()) {
      updateObjectGeometry(*graph.mesh(), attrs);
    }
  }

  for (const auto& node_id : merged_nodes) {
    curr_active.erase(node_id);
  }
}

std::unordered_set<NodeId> MeshSegmenter::getActiveNodes() const {
  std::unordered_set<NodeId> active_nodes;
  for (const auto& label_nodes_pair : active_nodes_) {
    active_nodes.insert(label_nodes_pair.second.begin(), label_nodes_pair.second.end());
  }
  return active_nodes;
}

void MeshSegmenter::updateNodeInGraph(DynamicSceneGraph& graph,
                                      const Cluster& cluster,
                                      const SceneGraphNode& node,
                                      uint64_t timestamp) {
  auto& attrs = node.attributes<ObjectNodeAttributes>();
  attrs.last_update_time_ns = timestamp;
  attrs.is_active = true;

  mergeList(attrs.mesh_connections, cluster.indices);
  updateObjectGeometry(*graph.mesh(), attrs);
}

void MeshSegmenter::addNodeToGraph(DynamicSceneGraph& graph,
                                   const Cluster& cluster,
                                   uint32_t label,
                                   uint64_t timestamp) {
  if (cluster.indices.empty()) {
    LOG(ERROR) << "Encountered empty cluster with label" << static_cast<int>(label)
               << " @ " << timestamp << "[ns]";
    return;
  }

  auto attrs = std::make_unique<ObjectNodeAttributes>();
  attrs->last_update_time_ns = timestamp;
  attrs->is_active = true;
  attrs->semantic_label = label;
  attrs->mesh_connections.insert(
      attrs->mesh_connections.begin(), cluster.indices.begin(), cluster.indices.end());

  updateObjectGeometry(*graph.mesh(), *attrs, nullptr, config.bounding_box_type);

  graph.emplaceNode(config.layer_id, next_node_id_, std::move(attrs));
  active_nodes_.at(label).insert(next_node_id_);
  ++next_node_id_;
}

}  // namespace hydra
