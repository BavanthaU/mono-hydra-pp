/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VisionImuFrontendParams.cpp
 * @brief  Class to parse, print, and store the parameters of the frontend.
 * @author Antoni Rosinol
 */

#include "kimera-vio/frontend/VisionImuFrontendParams.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <string>
#include <utility>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/utils/UtilsNumerical.h"
#include "kimera-vio/utils/YamlParser.h"

DEFINE_bool(visualize_feature_tracks, true, "Display feature tracks.");
DEFINE_bool(visualize_frontend_images,
            false,
            "Display images in Frontend logger for debugging (only use "
            "if in sequential mode, otherwise expect segfaults). ");
DEFINE_bool(save_frontend_images,
            false,
            "Save images in Frontend logger to disk for debugging (only use "
            "if in sequential mode, otherwise expect segfaults). ");
DEFINE_bool(log_feature_tracks, false, "Display/Save feature tracks images.");
DEFINE_bool(log_mono_tracking_images, false, "Display/Save mono tracking.");
DEFINE_bool(log_stereo_matching_images,
            false,
            "Display/Save stereo tracking rectified and unrectified images.");

namespace VIO {

FrontendParams::FrontendParams() : PipelineParams("Frontend Parameters") {}

void FrontendParams::print() const {
  std::stringstream out;
  PipelineParams::print(out,
                        // Tracker params
                        // "** STEREO tracker parameters **\n"
                        "min_intra_keyframe_time_: ",
                        min_intra_keyframe_time_ns_,
                        "max_intra_keyframe_time_: ",
                        max_intra_keyframe_time_ns_,
                        "minNumberFeatures_: ",
                        min_number_features_,
                        "useStereoTracking_: ",
                        use_stereo_tracking_,
                        "rgbd_use_depth_only_frontend_: ",
                        rgbd_use_depth_only_frontend_,
                        "max_disparity_since_lkf_: ",
                        max_disparity_since_lkf_,
                        "useRANSAC_: ",
                        useRANSAC_,
                        "Use 2D2D Tracking",
                        use_2d2d_tracking_,
                        "Use 3D3D Tracking",
                        use_3d3d_tracking_,
                        "Use PnP Tracking",
                        use_pnp_tracking_,
                        "use_depth_stride_: ",
                        use_depth_stride_,
                        "depth_stride_: ",
                        depth_stride_,
                        "rgbd_sparse_depth_max_meas_: ",
                        rgbd_sparse_depth_max_meas_,
                        "rgbd_use_dense_depth_factors_: ",
                        rgbd_use_dense_depth_factors_,
                        "rgbd_dense_depth_max_meas_: ",
                        rgbd_dense_depth_max_meas_);
  LOG(INFO) << out.str();

  feature_detector_params_.print();

  tracker_params_.print();

  if (use_stereo_tracking_) {
    stereo_matching_params_.print();
  }
}

bool FrontendParams::parseYAML(const std::string& filepath) {
  feature_detector_params_.parseYAML(filepath);
  tracker_params_.parseYAML(filepath);
  stereo_matching_params_.parseYAML(filepath);

  YamlParser yaml_parser(filepath);

  // Given in seconds, needs to be converted to nanoseconds.
  double min_intra_keyframe_time_seconds;
  yaml_parser.getYamlParam("min_intra_keyframe_time", &min_intra_keyframe_time_seconds);
  min_intra_keyframe_time_ns_ =
      UtilsNumerical::SecToNsec(min_intra_keyframe_time_seconds);

  double max_intra_keyframe_time_seconds;
  yaml_parser.getYamlParam("max_intra_keyframe_time", &max_intra_keyframe_time_seconds);
  max_intra_keyframe_time_ns_ =
      UtilsNumerical::SecToNsec(max_intra_keyframe_time_seconds);

  int min_number_features;
  yaml_parser.getYamlParam("minNumberFeatures", &min_number_features);
  min_number_features_ = static_cast<size_t>(min_number_features);
  yaml_parser.getYamlParam("useStereoTracking", &use_stereo_tracking_);
  if (yaml_parser.hasParam("rgbd_use_depth_only_frontend")) {
    yaml_parser.getYamlParam("rgbd_use_depth_only_frontend",
                             &rgbd_use_depth_only_frontend_);
  }
  yaml_parser.getYamlParam("useRANSAC", &useRANSAC_);
  yaml_parser.getYamlParam("use_2d2d_tracking", &use_2d2d_tracking_);
  yaml_parser.getYamlParam("use_3d3d_tracking", &use_3d3d_tracking_);
  yaml_parser.getYamlParam("use_pnp_tracking", &use_pnp_tracking_);
  yaml_parser.getYamlParam("max_disparity_since_lkf", &max_disparity_since_lkf_);

  if (yaml_parser.hasParam("use_depth_stride")) {
    yaml_parser.getYamlParam("use_depth_stride", &use_depth_stride_);
  }
  if (yaml_parser.hasParam("depth_stride")) {
    int depth_stride = 0;
    yaml_parser.getYamlParam("depth_stride", &depth_stride);
    depth_stride_ = depth_stride > 0 ? static_cast<size_t>(depth_stride) : 1u;
  }
  if (yaml_parser.hasParam("use_depth_gyro_gate")) {
    yaml_parser.getYamlParam("use_depth_gyro_gate", &use_depth_gyro_gate_);
  }
  if (yaml_parser.hasParam("depth_gyro_threshold_radps")) {
    yaml_parser.getYamlParam("depth_gyro_threshold_radps",
                             &depth_gyro_threshold_radps_);
  }
  if (yaml_parser.hasParam("rgbd_sparse_depth_max_meas")) {
    int depth_max_meas = 0;
    yaml_parser.getYamlParam("rgbd_sparse_depth_max_meas", &depth_max_meas);
    rgbd_sparse_depth_max_meas_ =
        depth_max_meas > 0 ? static_cast<size_t>(depth_max_meas) : 0u;
  }
  if (yaml_parser.hasParam("rgbd_use_dense_depth_factors")) {
    yaml_parser.getYamlParam("rgbd_use_dense_depth_factors",
                             &rgbd_use_dense_depth_factors_);
  }
  if (yaml_parser.hasParam("rgbd_dense_depth_max_meas")) {
    int depth_max_meas = 0;
    yaml_parser.getYamlParam("rgbd_dense_depth_max_meas", &depth_max_meas);
    rgbd_dense_depth_max_meas_ =
        depth_max_meas > 0 ? static_cast<size_t>(depth_max_meas) : 0u;
  }
  
  // TODO(Toni): use yaml at some point
  visualize_feature_tracks_ = FLAGS_visualize_feature_tracks;
  visualize_frontend_images_ = FLAGS_visualize_frontend_images;
  save_frontend_images_ = FLAGS_save_frontend_images;
  log_feature_tracks_ = FLAGS_log_feature_tracks;
  log_mono_tracking_images_ = FLAGS_log_mono_tracking_images;
  log_stereo_matching_images_ = FLAGS_log_stereo_matching_images;

  return true;
}

bool FrontendParams::equals(const FrontendParams& tp2, double tol) const {
  return tracker_params_.equals(tp2.tracker_params_, tol) &&
         // stereo matching
         stereo_matching_params_.equals(tp2.stereo_matching_params_, tol) &&
         // STEREO parameters:
         (fabs(max_intra_keyframe_time_ns_ - tp2.max_intra_keyframe_time_ns_) <= tol) &&
         (fabs(min_intra_keyframe_time_ns_ - tp2.min_intra_keyframe_time_ns_) <= tol) &&
         (min_number_features_ == tp2.min_number_features_) &&
         (fabs(max_disparity_since_lkf_ - tp2.max_disparity_since_lkf_) <= tol) &&
         (use_stereo_tracking_ == tp2.use_stereo_tracking_) &&
         (rgbd_use_depth_only_frontend_ == tp2.rgbd_use_depth_only_frontend_) &&
         (use_depth_stride_ == tp2.use_depth_stride_) &&
         (depth_stride_ == tp2.depth_stride_) &&
         (use_depth_gyro_gate_ == tp2.use_depth_gyro_gate_) &&
         (fabs(depth_gyro_threshold_radps_ - tp2.depth_gyro_threshold_radps_) <= tol) &&
         (rgbd_sparse_depth_max_meas_ == tp2.rgbd_sparse_depth_max_meas_) &&
         (rgbd_use_dense_depth_factors_ == tp2.rgbd_use_dense_depth_factors_) &&
         (rgbd_dense_depth_max_meas_ == tp2.rgbd_dense_depth_max_meas_);
}

}  // namespace VIO
