/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   RgbdVisionImuFrontend.cpp
 * @brief  Class describing an RGBD tracker
 * @author Nathan Hughes
 */

#include "kimera-vio/frontend/RgbdVisionImuFrontend.h"

#include <algorithm>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtsam/geometry/Rot3.h>

#include "kimera-vio/utils/Timer.h"
#include "kimera-vio/utils/UtilsNumerical.h"
#include "kimera-vio/backend/DepthMeasurements.h"

DEFINE_bool(log_rgbd_tracking_images,
            false,
            "Display and/or save rgbd specific debug images");
DECLARE_bool(do_fine_imu_camera_temporal_sync);

namespace VIO {

using RgbdOutputPtr = FrontendOutputPacketBase::UniquePtr;
using RgbdInputPtr = FrontendInputPacketBase::UniquePtr;

using utils::Timer;

RgbdVisionImuFrontend::RgbdVisionImuFrontend(
    const FrontendParams& frontend_params,
    const ImuParams& imu_params,
    const ImuBias& imu_initial_bias,
    const RgbdCamera::ConstPtr& camera,
    DisplayQueue* display_queue,
    bool log_output,
    std::optional<OdometryParams> odom_params)
    : VisionImuFrontend(frontend_params,
                        imu_params,
                        imu_initial_bias,
                        display_queue,
                        log_output,
                        odom_params),
      camera_(camera),
      use_depth_stride_(frontend_params.use_depth_stride_),
      depth_stride_(std::max<size_t>(1u, frontend_params.depth_stride_)),
      use_depth_gyro_gate_(frontend_params.use_depth_gyro_gate_),
      depth_gyro_threshold_radps_(frontend_params.depth_gyro_threshold_radps_),
      use_depth_only_frontend_(
          frontend_params.rgbd_use_depth_only_frontend_),
      sparse_depth_max_meas_(frontend_params.rgbd_sparse_depth_max_meas_),
      use_dense_depth_factors_(frontend_params.rgbd_use_dense_depth_factors_),
      dense_depth_max_meas_(frontend_params.rgbd_dense_depth_max_meas_) {
  CHECK(camera_);

  feature_detector_ = std::make_unique<FeatureDetector>(
      frontend_params.feature_detector_params_);

  tracker_ = std::make_unique<Tracker>(
      frontend_params_.tracker_params_, camera_, display_queue);

  if (use_depth_only_frontend_) {
    LOG(INFO) << "RGBD frontend depth-only mode enabled: "
                 "pseudo-stereo tracking/measurements disabled.";
  }
  if (use_dense_depth_factors_ && !use_depth_only_frontend_) {
    LOG(WARNING)
        << "rgbd_use_dense_depth_factors is enabled, but "
           "rgbd_use_depth_only_frontend is false; dense depth measurements "
           "are only used in depth-only mode.";
  }

  if (VLOG_IS_ON(1)) tracker_->tracker_params_.print();
}

RgbdVisionImuFrontend::~RgbdVisionImuFrontend() {
  LOG(INFO) << "RgbdVisionImuFrontend destructor called.";
}

RgbdOutputPtr RgbdVisionImuFrontend::bootstrapSpin(RgbdInputPtr&& base_input) {
  CHECK(frontend_state_ == FrontendState::Bootstrap);
  CHECK(base_input);
  auto input = castUnique<RgbdImuSyncPacket>(std::move(base_input));
  CHECK(input) << "input was not RGBD packet";

  // Initialize members of the Frontend
  processFirstFrame(*input->rgbd_frame_);
  frontend_state_ = FLAGS_do_fine_imu_camera_temporal_sync
                        ? FrontendState::InitialTimeAlignment
                        : FrontendState::Nominal;

  if (!FLAGS_do_fine_imu_camera_temporal_sync && odom_params_) {
    // we assume that the first frame is hardcoded to be a keyframe.
    // it's okay if world_NavState_odom_ is none (it gets cached later)
    cacheExternalOdometry(input.get());
  }

  if (FLAGS_do_fine_imu_camera_temporal_sync) {
    return nullptr;  // skip adding a frame to all downstream modules
  }

  CHECK(frame_lkf_);
  SparseDepthMeasurements sparse_depth_meas;
  return std::make_unique<RgbdFrontendOutput>(frame_lkf_->isKeyframe(),
                                              nullptr,
                                              camera_->getBodyPoseCam(),
                                              *frame_lkf_,
                                              *input->rgbd_frame_,
                                              nullptr,
                                              input->imu_accgyrs_,
                                              cv::Mat(),
                                              getTrackerInfo(),
                                              sparse_depth_meas);
}

RgbdOutputPtr RgbdVisionImuFrontend::nominalSpin(RgbdInputPtr&& base_input) {
  CHECK(frontend_state_ == FrontendState::Nominal ||
        frontend_state_ == FrontendState::InitialTimeAlignment);
  CHECK(base_input);
  auto input = castUnique<RgbdImuSyncPacket>(std::move(base_input));
  CHECK(input);

  utils::StatsCollector timing_stats_frame_rate("VioFrontend Frame Rate [ms]");
  utils::StatsCollector timing_stats_kf_rate("VioFrontend Keyframe Rate [ms]");
  utils::StatsCollector timing_stats_pim("IMU Preintegration Timing [us]");
  auto start_time = Timer::tic();

  const RgbdFrame& frame_k = *input->rgbd_frame_;
  const auto& k = frame_k.id_;
  VLOG(1) << "------------------- Processing frame k = " << k
          << "--------------------";

  if (VLOG_IS_ON(10)) input->print();

  // see stereo frontend for PIM window end behavior explanation
  auto tic_pim = Timer::tic();
  const ImuFrontend::PimPtr& pim = imu_frontend_->preintegrateImuMeasurements(
      input->imu_stamps_, input->imu_accgyrs_);
  CHECK(pim);

  auto pim_duration = Timer::toc<std::chrono::microseconds>(tic_pim).count();
  VLOG(1) << "Current IMU Preintegration time: " << pim_duration << "[us]";

  const gtsam::Rot3 body_Rot_cam = camera_->getBodyPoseCam().rotation();
  const gtsam::Rot3 cam_Rot_body = body_Rot_cam.inverse();

  // Relative rotation of the left cam rectified from the last keyframe to the
  // curr frame (pim.deltaRij() corresponds to bodyLkf_R_bodyK_imu)
  gtsam::Rot3 camLrectLkf_R_camLrectK_imu =
      cam_Rot_body * pim->deltaRij() * body_Rot_cam;

  if (VLOG_IS_ON(10)) {
    body_Rot_cam.print("Body_Rot_cam");
    camLrectLkf_R_camLrectK_imu.print("calLrectLkf_R_camLrectK_imu");
  }

  VLOG(10) << "Starting processStereoFrame...";
  cv::Mat feature_tracks;
  StatusStereoMeasurementsPtr stereo_measurements =
      processFrame(frame_k,
                   camLrectLkf_R_camLrectK_imu,
                   input->imu_accgyrs_,
                   &feature_tracks);
  VLOG(10) << "Finished processStereoFrame.";

  if (VLOG_IS_ON(5)) {
    RgbdVisionImuFrontend::printStatusStereoMeasurements(*stereo_measurements);
  }

  if (frame_km1_->isKeyframe()) {
    checkAndLogKeyframe();
    VLOG(1) << "Keyframe " << frame_km1_->id_
            << " with: " << stereo_measurements->second.size()
            << " smart measurements";

    VLOG(10) << "Reset IMU preintegration with latest IMU bias.";
    imu_frontend_->resetIntegrationWithCachedBias();

    timing_stats_kf_rate.AddSample(Timer::toc(start_time).count());
    VLOG(2) << "Frontend output is a keyframe: pushing to output callbacks.";
  } else {
    timing_stats_frame_rate.AddSample(Timer::toc(start_time).count());
    VLOG(2) << "Frontend output is not a keyframe. Skipping output queue push.";
  }

  const bool is_keyframe = frame_km1_->isKeyframe();
  SparseDepthMeasurements sparse_depth_meas;
  if (is_keyframe) {
    const bool use_dense =
        use_depth_only_frontend_ && use_dense_depth_factors_;
    const size_t max_depth_meas =
        use_dense ? dense_depth_max_meas_ : sparse_depth_max_meas_;
    sparse_depth_meas = selectSparseDepth(*frame_km1_, max_depth_meas);
  }
  return std::make_unique<RgbdFrontendOutput>(
      is_keyframe && frontend_state_ == FrontendState::Nominal,
      stereo_measurements,
      camera_->getBodyPoseCam(),
      *frame_km1_,
      frame_k,
      pim,
      input->imu_accgyrs_,
      feature_tracks,
      getTrackerInfo(),
      sparse_depth_meas,
      is_keyframe ? getExternalOdometryRelativeBodyPose(input.get())
                  : std::nullopt,
      is_keyframe ? getExternalOdometryWorldVelocity(input.get())
                  : std::nullopt);
}

void RgbdVisionImuFrontend::processFirstFrame(const RgbdFrame& rgbd_frame) {
  VLOG(2) << "Processing first stereo frame \n";
  // turn rgbd into fake stereo (only lefy frame is valid)
  frame_km1_ = rgbd_frame.getStereoFrame();
  frame_lkf_ = frame_km1_;
  frame_km1_->setIsKeyframe(true);

  Frame* left_frame = &frame_km1_->left_frame_;
  CHECK_EQ(left_frame->keypoints_.size(), 0)
      << "Keypoints present in first frame: do not extract keypoints manually";

  last_keyframe_timestamp_ = frame_km1_->timestamp_;
  last_frame_timestamp_ = last_keyframe_timestamp_;

  // no need for retification during detection: right frame is hallucinated
  CHECK(feature_detector_);
  cv::Mat depth_mask =
      rgbd_frame.depth_img_.getDetectionMask(camera_->getCamParams());
  if (!left_frame->detection_mask_.empty()) {
    cv::bitwise_and(left_frame->detection_mask_, depth_mask,
                    left_frame->detection_mask_);
  } else {
    left_frame->detection_mask_ = depth_mask;
  }
  applySemanticMask(&left_frame->detection_mask_);
  feature_detector_->featureDetection(left_frame);

  // we put undistorted keypoints in the stereo container (as we reuse some of
  // the stereo pipeline)
  camera_->undistortKeypoints(left_frame->keypoints_,
                              &frame_km1_->left_keypoints_rectified_);

  // make features in fake stereo frame corresponding to depth in rgbd
  // No IMU samples yet here; only stride gate applies on the very first frame.
  const bool depth_active = useDepthThisFrame(rgbd_frame.id_);
  if (VLOG_IS_ON(1)) {
    VLOG(1) << "RGBD first frame depth usage: "
            << (depth_active ? "enabled" : "skipped (stride/gyro)");
  }
  rgbd_frame.fillStereoFrame(*camera_, *frame_km1_, depth_active);
  // No depth reuse on first frame

  ++frame_count_;

  // reset current pim (so that it's valid for the time window starting with
  // this keyframe)
  imu_frontend_->resetIntegrationWithCachedBias();
}

void RgbdVisionImuFrontend::checkAndLogKeyframe() {
  CHECK_EQ(frame_lkf_->timestamp_, frame_km1_->timestamp_);
  CHECK_EQ(frame_lkf_->id_, frame_km1_->id_);
  CHECK(frame_lkf_->isKeyframe());

  if (logger_) {
    logger_->logFrontendStats(frame_km1_->timestamp_,
                              getTrackerInfo(),
                              tracker_status_summary_,
                              frame_km1_->left_frame_.getNrValidKeypoints());
    // Logger needs information in camera frame for evaluation
    logger_->logFrontendRansac(frame_km1_->timestamp_,
                               tracker_status_summary_.lkf_T_k_mono_,
                               tracker_status_summary_.lkf_T_k_stereo_);
  }
}

StatusStereoMeasurementsPtr RgbdVisionImuFrontend::processFrame(
    const RgbdFrame& rgbd_frame,
    const gtsam::Rot3& keyframe_R_cur_frame,
    const ImuAccGyrS& imu_accgyrs,
    cv::Mat* feature_tracks) {
  CHECK(tracker_);

  const Timestamp frame_diff_ns = rgbd_frame.timestamp_ - last_frame_timestamp_;
  last_frame_timestamp_ = rgbd_frame.timestamp_;
  VLOG(2) << "===================================================\n"
          << "Frame number: " << frame_count_ << " at time "
          << rgbd_frame.timestamp_
          << " timestamp diff [s]: " << UtilsNumerical::NsecToSec(frame_diff_ns)
          << " (timestamp diff [ns]: " << frame_diff_ns << ")";

  auto stereo_frame = rgbd_frame.getStereoFrame();

  VLOG(2) << "Starting feature tracking...";
  gtsam::Rot3 ref_frame_R_cur_frame =
      keyframe_R_ref_frame_.inverse().compose(keyframe_R_cur_frame);

  // no rectification matrix needed: we're hallucinating the right frame
  tracker_->featureTracking(&frame_km1_->left_frame_,
                            &stereo_frame->left_frame_,
                            ref_frame_R_cur_frame,
                            frontend_params_.feature_detector_params_);
  camera_->undistortKeypoints(stereo_frame->left_frame_.keypoints_,
                              &stereo_frame->left_keypoints_rectified_);

  if (feature_tracks) {
    // TODO(Toni) examine / fix ordering of debug image and outlier rejection
    *feature_tracks = tracker_->getTrackerImage(frame_km1_->left_frame_,
                                                stereo_frame->left_frame_);
  }
  VLOG(2) << "Finished feature tracking.";

  tracker_status_summary_.kfTrackingStatus_mono_ = TrackingStatus::INVALID;
  tracker_status_summary_.kfTrackingStatus_stereo_ = TrackingStatus::INVALID;

  StereoMeasurements stereo_measurements;

  // Choose whether to use depth on this frame (stride + optional gyro gate).
  const bool depth_active =
      useDepthThisFrame(rgbd_frame.id_) && gyroAllowsDepth(imu_accgyrs);

  // Only create keyframes when depth is active to ensure backend receives depth.
  const bool want_keyframe =
      depth_active &&
      shouldBeKeyframe(stereo_frame->left_frame_, frame_lkf_->left_frame_);

  if (want_keyframe) {
    handleKeyframe(rgbd_frame, *stereo_frame, keyframe_R_cur_frame);
    stereo_frame->setIsKeyframe(true);

    fillSmartStereoMeasurements(*stereo_frame, &stereo_measurements);

    frame_lkf_ = stereo_frame;
    last_keyframe_timestamp_ = stereo_frame->timestamp_;
    keyframe_R_ref_frame_ = gtsam::Rot3();
    ++keyframe_count_;
  } else {
    CHECK_EQ(stereo_measurements.size(), 0u);
    stereo_frame->setIsKeyframe(false);
    // Update rotation from keyframe to next iteration reference frame
    keyframe_R_ref_frame_ = keyframe_R_cur_frame;
  }

  // Reset frames.
  frame_km1_ = stereo_frame;
  ++frame_count_;
  return std::make_shared<StatusStereoMeasurements>(
      std::make_pair(tracker_status_summary_, stereo_measurements));
}

void RgbdVisionImuFrontend::handleKeyframe(
    const RgbdFrame& rgbd_frame,
    StereoFrame& frame,
    const gtsam::Rot3& keyframe_R_frame) {
  const bool depth_active = useDepthThisFrame(rgbd_frame.id_);
  const bool use_pseudo_stereo =
      frontend_params_.use_stereo_tracking_ && !use_depth_only_frontend_;

  if (frontend_params_.useRANSAC_) {
    // MONO geometric outlier rejection
    TrackingStatusPose status_mono;
    outlierRejectionMono(keyframe_R_frame,
                         &frame_lkf_->left_frame_,
                         &frame.left_frame_,
                         &status_mono);

    if (VLOG_IS_ON(1) && use_depth_stride_) {
      VLOG(1) << "RGBD keyframe " << rgbd_frame.id_
              << (depth_active ? " using depth." : " skipping depth (stride).");
    }

    rgbd_frame.fillStereoFrame(*camera_, frame, depth_active);

    TrackingStatusPose status_stereo;
    if (use_pseudo_stereo && depth_active) {
      outlierRejectionStereo(
          camera_->getFakeStereoCamera(),
          keyframe_R_frame,
          frame_lkf_.get(),
          &frame,
          &status_stereo,
          &tracker_status_summary_.infoMatStereoTranslation_);
    } else {
      status_stereo.first = TrackingStatus::DISABLED;
      status_stereo.second = gtsam::Pose3();
      tracker_status_summary_.infoMatStereoTranslation_.setZero();
    }

    TrackingStatusPose status_pnp;
    if (frontend_params_.use_pnp_tracking_) {
      outlierRejectionPnP(frame, &status_pnp);
    } else {
      status_pnp.first = TrackingStatus::INVALID;
      status_pnp.second = gtsam::Pose3();
    }

    tracker_status_summary_.kfTrackingStatus_stereo_ = status_stereo.first;
    tracker_status_summary_.lkf_T_k_stereo_ = status_stereo.second;
    tracker_status_summary_.kfTrackingStatus_mono_ = status_mono.first;
    tracker_status_summary_.lkf_T_k_mono_ = status_mono.second;
    tracker_status_summary_.kfTracking_status_pnp_ = status_pnp.first;
    tracker_status_summary_.W_T_k_pnp_ = status_pnp.second;
  } else {
    tracker_status_summary_.kfTrackingStatus_mono_ = TrackingStatus::DISABLED;
    tracker_status_summary_.kfTrackingStatus_stereo_ = TrackingStatus::DISABLED;
    tracker_status_summary_.kfTracking_status_pnp_ = TrackingStatus::DISABLED;
  }

  if (VLOG_IS_ON(2)) {
    printTrackingStatus(tracker_status_summary_.kfTrackingStatus_mono_, "mono");
    printTrackingStatus(tracker_status_summary_.kfTrackingStatus_stereo_,
                        "stereo");
  }

  // we don't use a rectification matrix for detection: right frame is
  // hallucinated
  CHECK(feature_detector_);
  cv::Mat depth_mask =
      rgbd_frame.depth_img_.getDetectionMask(camera_->getCamParams());
  if (!frame.left_frame_.detection_mask_.empty()) {
    cv::bitwise_and(frame.left_frame_.detection_mask_, depth_mask,
                    frame.left_frame_.detection_mask_);
  } else {
    frame.left_frame_.detection_mask_ = depth_mask;
  }
  applySemanticMask(&frame.left_frame_.detection_mask_);
  feature_detector_->featureDetection(&frame.left_frame_);
  camera_->undistortKeypoints(frame.left_frame_.keypoints_,
                              &frame.left_keypoints_rectified_);

  rgbd_frame.fillStereoFrame(*camera_, frame, depth_active);

  // log debugging info
  logTrackingImages(frame, rgbd_frame);
  frame.checkStatusRightKeypoints(&tracker_->debug_info_);
}

void RgbdVisionImuFrontend::fillSmartStereoMeasurements(
    const StereoFrame& frame,
    StereoMeasurements* measurements) const {
  CHECK(tracker_);
  CHECK_NOTNULL(measurements);
  frame.checkStereoFrame();

  const LandmarkIds& landmark_ids = frame.left_frame_.landmarks_;
  const StatusKeypointsCV& left_keypoints = frame.left_keypoints_rectified_;
  const StatusKeypointsCV& right_keypoints = frame.right_keypoints_rectified_;
  const bool use_pseudo_stereo =
      frontend_params_.use_stereo_tracking_ && !use_depth_only_frontend_;

  measurements->clear();
  measurements->reserve(landmark_ids.size());
  for (size_t i = 0; i < landmark_ids.size(); ++i) {
    if (landmark_ids[i] == -1) {
      continue;  // skip invalid points
    }

    const double uL = left_keypoints.at(i).second.x;
    const double v = left_keypoints.at(i).second.y;
    // TODO(nathan) should we just continue here?
    double uR = std::numeric_limits<double>::quiet_NaN();
    if (use_pseudo_stereo &&
        right_keypoints.at(i).first == KeypointStatus::VALID) {
      uR = right_keypoints.at(i).second.x;
    }

    measurements->push_back(
        std::make_pair(landmark_ids[i], gtsam::StereoPoint2(uL, uR, v)));
  }
}

SparseDepthMeasurements RgbdVisionImuFrontend::selectSparseDepth(
    const StereoFrame& frame, size_t max_meas) const {
  SparseDepthMeasurements out;
  if (max_meas == 0) {
    return out;
  }
  const auto& depths = frame.keypoints_depth_;
  const auto& left_keypoints = frame.left_keypoints_rectified_;
  const auto& lmks = frame.left_frame_.landmarks_;
  const auto& img = frame.left_frame_.img_;
  CHECK_EQ(depths.size(), left_keypoints.size());
  CHECK_EQ(depths.size(), lmks.size());
  CHECK_EQ(img.type(), CV_8UC1);

  std::vector<std::pair<double, size_t>> scored_indices;
  scored_indices.reserve(depths.size());

  // Precompute gradients for scoring.
  cv::Mat grad_x, grad_y;
  cv::Sobel(img, grad_x, CV_32F, 1, 0, 3);
  cv::Sobel(img, grad_y, CV_32F, 0, 1, 3);

  for (size_t i = 0; i < depths.size(); ++i) {
    if (lmks[i] == -1) continue;
    const double d = depths[i];
    if (d <= 0.0 || !std::isfinite(d)) continue;
    if (left_keypoints[i].first != KeypointStatus::VALID) continue;
    const auto& kp = left_keypoints[i].second;
    const int u = static_cast<int>(std::round(kp.x));
    const int v = static_cast<int>(std::round(kp.y));
    if (u < 1 || v < 1 || u >= grad_x.cols - 1 || v >= grad_x.rows - 1) {
      continue;
    }
    const float gx = grad_x.at<float>(v, u);
    const float gy = grad_y.at<float>(v, u);
    const double grad_mag = std::hypot(gx, gy);
    scored_indices.emplace_back(grad_mag, i);
  }

  std::sort(scored_indices.begin(),
            scored_indices.end(),
            [](const auto& a, const auto& b) { return a.first > b.first; });

  out.reserve(std::min(max_meas, scored_indices.size()));
  for (const auto& scored : scored_indices) {
    if (out.size() >= max_meas) break;
    const size_t i = scored.second;
    const auto& kp = left_keypoints[i].second;
    out.push_back({lmks[i], depths[i], kp.x, kp.y});
  }
  return out;
}

cv::Mat drawDepthImage(const StereoFrame& stereo_frame,
                       const DepthFrame& depth_frame,
                       const CameraParams& params) {
  cv::Mat depth_img;
  // convert to uint8_t while so that [0, max_depth] maps to 255
  depth_frame.depth_img_.convertTo(
      depth_img,
      CV_8UC1,
      params.depth.depth_to_meters_ / params.depth.max_depth_ * 255);

  cv::Mat img_rgb;
  cv::cvtColor(depth_img, img_rgb, cv::COLOR_GRAY2RGB);

  static const cv::Scalar red(0, 0, 255);
  static const cv::Scalar green(0, 255, 0);

  const Frame& frame = stereo_frame.left_frame_;
  // Add all keypoints in cur_frame with the tracks.
  for (size_t i = 0; i < frame.keypoints_.size(); ++i) {
    const cv::Point2f& px = frame.keypoints_.at(i);
    if (stereo_frame.left_keypoints_rectified_[i].first ==
            KeypointStatus::VALID &&
        stereo_frame.right_keypoints_rectified_[i].first ==
            KeypointStatus::VALID) {
      cv::circle(img_rgb, px, 4, green, 2);
    } else {
      cv::circle(img_rgb, px, 4, red, 2);
    }
  }
  return img_rgb;
}

void RgbdVisionImuFrontend::logTrackingImages(const StereoFrame& frame,
                                              const RgbdFrame& rgbd_frame) {
  const bool logger_valid = logger_ && (FLAGS_visualize_frontend_images ||
                                        FLAGS_save_frontend_images);
  if (logger_valid && FLAGS_log_mono_tracking_images) {
    sendMonoTrackingToLogger(frame);
  }

  const bool log_tracks = logger_valid && FLAGS_log_feature_tracks;
  const bool display_tracks = display_queue_ && FLAGS_visualize_feature_tracks;
  if (!log_tracks && !display_tracks) {
    return;
  }

  cv::Mat img =
      tracker_->getTrackerImage(frame_lkf_->left_frame_, frame.left_frame_);
  if (log_tracks) {
    logger_->logFrontendImg(frame.id_,
                            img,
                            "monoFeatureTracksLeft",
                            "/monoFeatureTracksLeftImg/",
                            FLAGS_visualize_frontend_images,
                            FLAGS_save_frontend_images);
  }

  if (logger_valid && FLAGS_log_rgbd_tracking_images) {
    cv::Mat rgbd_img =
        drawDepthImage(frame, rgbd_frame.depth_img_, camera_->getCamParams());
    logger_->logFrontendImg(frame.id_,
                            rgbd_img,
                            "rgbdDepthFeatures",
                            "/rgbdDepthFeaturesImg/",
                            FLAGS_visualize_frontend_images,
                            FLAGS_save_frontend_images);
  }

  if (display_tracks) {
    displayImage(frame.timestamp_, "feature_tracks", img, display_queue_);
  }
}

void RgbdVisionImuFrontend::sendMonoTrackingToLogger(
    const StereoFrame& frame) const {
  const Frame& cur_left_frame = frame.left_frame_;
  const Frame& ref_left_frame = frame_lkf_->left_frame_;

  // Find keypoint matches.
  DMatchVec matches;
  for (size_t i = 0; i < cur_left_frame.keypoints_.size(); ++i) {
    if (cur_left_frame.landmarks_.at(i) != -1) {  // if landmark is valid
      auto it = find(ref_left_frame.landmarks_.begin(),
                     ref_left_frame.landmarks_.end(),
                     cur_left_frame.landmarks_.at(i));
      if (it != ref_left_frame.landmarks_.end()) {  // if landmark was found
        int nPos = std::distance(ref_left_frame.landmarks_.begin(), it);
        matches.push_back(cv::DMatch(nPos, i, 0));
      }
    }
  }

  // Plot matches.
  cv::Mat img_left_lkf_kf =
      UtilsOpenCV::DrawCornersMatches(ref_left_frame.img_,
                                      ref_left_frame.keypoints_,
                                      cur_left_frame.img_,
                                      cur_left_frame.keypoints_,
                                      matches,
                                      false);  // true: random color
  cv::putText(img_left_lkf_kf,
              "M:" + std::to_string(keyframe_count_ - 1) + "-" +
                  std::to_string(keyframe_count_),
              KeypointCV(10, 15),
              CV_FONT_HERSHEY_COMPLEX,
              0.6,
              cv::Scalar(0, 255, 0));

  logger_->logFrontendImg(cur_left_frame.id_,
                          img_left_lkf_kf,
                          "monoTrackingUnrectified",
                          "/monoTrackingUnrectifiedImg/",
                          FLAGS_visualize_frontend_images,
                          FLAGS_save_frontend_images);
}

bool RgbdVisionImuFrontend::useDepthThisFrame(const FrameId& id) const {
  if (!use_depth_stride_) {
    return true;
  }
  return (id % depth_stride_) == 0u;
}

bool RgbdVisionImuFrontend::gyroAllowsDepth(
    const ImuAccGyrS& imu_accgyrs) const {
  if (!use_depth_gyro_gate_) {
    return true;
  }
  if (imu_accgyrs.cols() == 0) {
    return false;
  }
  // Compute max gyro magnitude over the window.
  double max_gyr = 0.0;
  for (int i = 0; i < imu_accgyrs.cols(); ++i) {
    const Eigen::Vector3d gyr = imu_accgyrs.block<3, 1>(3, i);
    max_gyr = std::max(max_gyr, gyr.norm());
  }
  return max_gyr < depth_gyro_threshold_radps_;
}

void RgbdVisionImuFrontend::applySemanticMask(cv::Mat* detection_mask) const {
  if (!semantic_mask_loaded_ || !detection_mask) {
    return;
  }
  if (semantic_mask_.size() != detection_mask->size()) {
    LOG_FIRST_N(WARNING, 1)
        << "Semantic mask size mismatch; skipping semantic masking.";
    return;
  }
  cv::bitwise_and(*detection_mask, semantic_mask_, *detection_mask);
}

gtsam::Pose3 RgbdVisionImuFrontend::getRelativePoseBodyMono() const {
  // left camera pose
  gtsam::Pose3 body_Pose_cam_ = camera_->getBodyPoseCam();
  return body_Pose_cam_ * tracker_status_summary_.lkf_T_k_mono_ *
         body_Pose_cam_.inverse();
}

gtsam::Pose3 RgbdVisionImuFrontend::getRelativePoseBodyStereo() const {
  // left camera pose
  gtsam::Pose3 body_Pose_cam_ = camera_->getBodyPoseCam();
  return body_Pose_cam_ * tracker_status_summary_.lkf_T_k_stereo_ *
         body_Pose_cam_.inverse();
}

void RgbdVisionImuFrontend::printStatusStereoMeasurements(
    const StatusStereoMeasurements& measurements) {
  LOG(INFO) << " SmartStereoMeasurements with status:";
  printTrackingStatus(measurements.first.kfTrackingStatus_mono_, "mono");
  printTrackingStatus(measurements.first.kfTrackingStatus_stereo_, "stereo");
  LOG(INFO) << " stereo points:";

  const StereoMeasurements& smartStereoMeas = measurements.second;
  for (const auto& smart_stereo_meas : smartStereoMeas) {
    LOG(INFO) << " " << smart_stereo_meas.second << " ";
  }
  LOG(INFO) << std::endl;
}

}  // namespace VIO
