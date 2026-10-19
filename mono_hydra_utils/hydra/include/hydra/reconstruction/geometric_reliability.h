#pragma once

#include "hydra/input/camera.h"

namespace hydra {

// A bounded consistency score, not a calibrated probability or depth variance.
// History belongs to the serial reconstruction worker, never voxel threads.
class GeometricReliability {
 public:
  struct Config {
    bool enabled = false;
    float min_factor = 0.2f;
    float scale_m = 0.05f;
    float scale_relative = 0.03f;
    float min_baseline_m = 0.02f;
    float max_baseline_m = 0.8f;
    double max_age_s = 1.5;
  };

  explicit GeometricReliability(const Config& config);
  cv::Mat compute(const InputData& data);
  void reset();

 private:
  const Config config_;
  const Sensor* sensor_ = nullptr;
  cv::Mat depth_;
  Eigen::Isometry3f world_T_camera_ = Eigen::Isometry3f::Identity();
  uint64_t timestamp_ns_ = 0;
};

void declare_config(GeometricReliability::Config& config);

}  // namespace hydra
