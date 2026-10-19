#include "hydra/reconstruction/geometric_reliability.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>

namespace hydra {
namespace {
bool usable(float z) { return std::isfinite(z) && z > 0.0f; }

// Do not interpolate across an occlusion boundary or missing depth.
bool smoothSample(const cv::Mat& depth, int u, int v, float scale) {
  const float z = depth.at<float>(v, u);
  if (!usable(z)) {
    return false;
  }
  for (const auto& d : {cv::Point(-1, 0), cv::Point(1, 0),
                        cv::Point(0, -1), cv::Point(0, 1)}) {
    const float neighbor = depth.at<float>(v + d.y, u + d.x);
    if (!usable(neighbor) || std::abs(neighbor - z) > 3.0f * scale) {
      return false;
    }
  }
  return true;
}
}  // namespace

void declare_config(GeometricReliability::Config& config) {
  using namespace config;
  name("GeometricReliability");
  field(config.enabled, "enabled");
  field(config.min_factor, "min_factor");
  field(config.scale_m, "scale_m");
  field(config.scale_relative, "scale_relative");
  field(config.min_baseline_m, "min_baseline_m");
  field(config.max_baseline_m, "max_baseline_m");
  field(config.max_age_s, "max_age_s");
  check(config.min_factor, GT, 0.0f, "min_factor");
  check(config.min_factor, LE, 1.0f, "min_factor");
  check(config.scale_m, GT, 0.0f, "scale_m");
  check(config.scale_relative, GE, 0.0f, "scale_relative");
  check(config.min_baseline_m, GE, 0.0f, "min_baseline_m");
  check(config.max_baseline_m, GT, config.min_baseline_m, "max_baseline_m");
  check(config.max_age_s, GT, 0.0, "max_age_s");
}

GeometricReliability::GeometricReliability(const Config& config)
    : config_(config::checkValid(config)) {}

void GeometricReliability::reset() {
  depth_.release();
  sensor_ = nullptr;
}

cv::Mat GeometricReliability::compute(const InputData& data) {
  if (!config_.enabled) {
    return {};
  }
  const auto* camera = dynamic_cast<const Camera*>(&data.getSensor());
  const auto pose = data.getSensorPose().cast<float>();
  if (!camera || data.depth_image.type() != CV_32FC1 ||
      data.depth_image.rows < 3 || data.depth_image.cols < 3 ||
      !pose.matrix().allFinite()) {
    reset();
    return {};
  }
  const auto& k = camera->getConfig();
  if (data.depth_image.rows != k.height || data.depth_image.cols != k.width) {
    reset();
    return {};
  }
  cv::Mat score(data.depth_image.size(), CV_32FC1, cv::Scalar(1.0f));
  const bool ordered = data.timestamp_ns > timestamp_ns_;
  const double age = ordered ? (data.timestamp_ns - timestamp_ns_) * 1.0e-9 : 0;
  const Eigen::Isometry3f previous_T_current = world_T_camera_.inverse() * pose;
  const float baseline = previous_T_current.translation().norm();
  const bool comparable = sensor_ == camera && !depth_.empty() && ordered &&
                          age <= config_.max_age_s &&
                          baseline >= config_.min_baseline_m &&
                          baseline <= config_.max_baseline_m;
  size_t supported = 0;
  double score_sum = 0;
  if (comparable) {
    for (int v = 1; v < k.height - 1; ++v) {
      for (int u = 1; u < k.width - 1; ++u) {
        const float z = data.depth_image.at<float>(v, u);
        if (!usable(z)) {
          continue;
        }
        const float scale = config_.scale_m + config_.scale_relative * z;
        if (!smoothSample(data.depth_image, u, v, scale)) {
          continue;
        }
        const Eigen::Vector3f p = previous_T_current *
            Eigen::Vector3f((u - k.cx) * z / k.fx, (v - k.cy) * z / k.fy, z);
        if (!p.allFinite() || p.z() <= 0.0f) {
          continue;
        }
        const float x = k.fx * p.x() / p.z() + k.cx;
        const float y = k.fy * p.y() / p.z() + k.cy;
        if (!std::isfinite(x) || !std::isfinite(y) || x < 1 || y < 1 ||
            x >= k.width - 2 || y >= k.height - 2) {
          continue;
        }
        const int px = std::lround(x), py = std::lround(y);
        if (!smoothSample(depth_, px, py, scale)) {
          continue;
        }
        const float old_z = depth_.at<float>(py, px);
        // A point behind an old foreground surface was not observed there.
        // Conversely, a newly exposed foreground is not evidence of bad depth.
        const float residual = p.z() - old_z;
        if (std::abs(residual) > 3.0f * scale) {
          continue;
        }
        // Include half-pixel footprint uncertainty on sloping surfaces.
        const float slope = 0.5f * (
            std::abs(depth_.at<float>(py, px + 1) - depth_.at<float>(py, px - 1)) +
            std::abs(depth_.at<float>(py + 1, px) - depth_.at<float>(py - 1, px)));
        const float normalized = residual / (scale + slope);
        const float factor = std::max(config_.min_factor,
                                      1.0f / (1.0f + normalized * normalized));
        score.at<float>(v, u) = factor;
        score_sum += factor;
        ++supported;
      }
    }
  }
  VLOG(2) << "GeometricReliability stamp=" << data.timestamp_ns
          << " supported=" << supported << " pixels=" << score.total()
          << " mean_supported=" << (supported ? score_sum / supported : 1.0);
  // Keep a small-baseline reference until translation accumulates. Reset on
  // clock rewind, stale history, sensor change, or excessive motion.
  if (!comparable && sensor_ == camera && !depth_.empty() && ordered &&
      age <= config_.max_age_s && baseline < config_.min_baseline_m) {
    return score;
  }
  depth_ = data.depth_image.clone();
  world_T_camera_ = pose;
  timestamp_ns_ = data.timestamp_ns;
  sensor_ = camera;
  return score;
}
}  // namespace hydra
