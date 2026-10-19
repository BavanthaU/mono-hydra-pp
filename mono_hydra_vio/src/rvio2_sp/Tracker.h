/**
* This file is part of R-VIO2.
*
* Copyright (C) 2022 Zheng Huai <zhuai@udel.edu> and Guoquan Huang <ghuang@udel.edu>
* For more information see <http://github.com/rpng/R-VIO2>
*
* R-VIO2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* R-VIO2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with R-VIO2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef RVIO2SP_TRACKER_H
#define RVIO2SP_TRACKER_H

#include <fstream>
#include <list>
#include <memory>
#include <vector>
#include <utility>
#include <unordered_map>

#include <Eigen/Core>

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include "Ransac.h"
#include "Feature.h"
#include "InputBuffer.h"
#include "FeatureDetector.h"
#include "SuperPoint.h"


namespace RVIO2SP
{

enum Type
{
    // Original values
    // 0: Init slam - reach the max. tracking length
    // 1: Pose only - reach the max. tracking length
    // 2: Pose only - lose track
    // 3: Exploration - local SLAM feature
    INIT_SLAM, POSE_ONLY_M, POSE_ONLY, EXPLO,
    // Return values
    // 5: Unused measurement
    // 6: Bad measurement
    UNUSED, BAD
};


class Tracker
{
public:

    Tracker(const cv::FileStorage& fsSettings);

    ~Tracker();

    void track(const int nImageId,
               const cv::Mat& image,
               const Eigen::Matrix3f& RcG,
               const Eigen::Vector3f& tcG,
               int nMapPtsNeeded,
               std::unordered_map<int,Feature*>& mFeatures);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

    bool start(const int nImageId,
               const cv::Mat& image,
               const Eigen::Matrix3f& RcG,
               const Eigen::Vector3f& tcG,
               std::unordered_map<int,Feature*>& mFeatures);

    void manage(const int nImageId,
                const cv::Mat& image,
                const Eigen::Matrix3f& RcG,
                const Eigen::Vector3f& tcG,
                const std::unordered_map<int,Feature*>& mFeatures);

    void preprocess(const int nImageId,
                    const cv::Mat& image,
                    const Eigen::Matrix3f& RcG,
                    const Eigen::Vector3f& tcG);

    void undistort(const std::vector<cv::Point2f>& src,
                   std::vector<cv::Point2f>& dst);

    void VisualTracking(const int nImageId,
                        const cv::Mat image,
                        int nMapPtsNeeded,
                        std::unordered_map<int,Feature*>& mFeatures);

    void DisplayTrack(const int nImageId,
                      const cv::Mat& image,
                      const std::vector<cv::Point2f>& vPrevFeatUVs,
                      const std::vector<cv::Point2f>& vCurrFeatUVs,
                      const std::vector<unsigned char>& vInlierFlags,
                      const std::vector<unsigned char>& vSources,
                      cv_bridge::CvImage& imOut);

    // Feature detection: Shi-Tomasi (classical) or SuperPoint keypoints
    int Detect(const cv::Mat& image, const int nCorners, std::vector<cv::Point2f>& vCorners);

    // Frame-to-frame association producing one point and flag per tracked feature
    void TrackKLT(const cv::Mat& image,
                  std::vector<cv::Point2f>& vFeatPts,
                  std::vector<unsigned char>& vInlierFlags,
                  std::vector<unsigned char>& vSources);

    void TrackSuperPoint(const cv::Mat& image,
                         std::vector<cv::Point2f>& vFeatPts,
                         std::vector<unsigned char>& vInlierFlags,
                         std::vector<unsigned char>& vSources);

    // Dense descriptor search around the prediction, then LK alignment and descriptor check
    void TrackSuperPointDense(const cv::Mat& image,
                              std::vector<cv::Point2f>& vFeatPts,
                              std::vector<unsigned char>& vInlierFlags,
                              std::vector<unsigned char>& vSources);

    // Rotation-only prediction of the tracked points in the current image
    void PredictWithGyro(const std::vector<cv::Point2f>& vPrevPts,
                         std::vector<cv::Point2f>& vPredPts);

    void AppendDescriptor(const cv::Point2f& pt, std::vector<float>& vDescs);

    void LogStats(const int nImageId);

    void DisplayNewer(const int nImageId,
                      const cv::Mat& image,
                      const std::vector<cv::Point2f>& vRefFeatUVs,
                      const std::vector<cv::Point2f>& vNewFeatUVs,
                      cv_bridge::CvImage& imOut);

    inline void OrienVec(const cv::Point2f& pt, Eigen::Vector3f& e)
    {
        float phi = atan2(pt.y, sqrt(pow(pt.x,2)+1));
        float psi = atan2(pt.x, 1);
        e << cos(phi)*sin(psi), sin(phi), cos(phi)*cos(psi);
    }

    inline float Parallax(const cv::Point2f& pt0, const cv::Point2f& ptk)
    {
        Eigen::Vector3f e0, ek;
        OrienVec(pt0, e0);
        OrienVec(ptk, ek);
        float theta = fabs(acos(ek.dot(mRx*e0)));
        return 40*sin(theta)>1 ? theta*180/M_PI : 0;
    }

public:

    std::vector<std::pair<int,Type> > mvFeatInfoForInitSlam;
    std::vector<std::vector<cv::Point2f> > mvvFeatMeasForInitSlam;

    std::vector<std::pair<int,Type> > mvFeatInfoForPoseOnly;
    std::vector<std::vector<cv::Point2f> > mvvFeatMeasForPoseOnly;

    std::vector<std::pair<int,cv::Point2f> > mvFeatMeasForExploration;

private:

    bool mbIsRGB;
    bool mbIsFisheye;

    bool mbRestartVT;
    bool mbRefreshVT;

    bool mbEnableSlam;
    bool mbEnableFilter;
    bool mbEnableEqualizer;

    int mnMaxFeatsPerImage;

    int mnMinTrackingLength;
    int mnMaxTrackingLength;

    float mnGoodParallax;

    cv::Mat mLastImage;

    cv::Mat mK;
    cv::Mat mD;

    Eigen::Matrix3f mRx;
    Eigen::Matrix3f mRr;

    std::list<Eigen::Matrix3f> mlCamOrientations;
    std::list<Eigen::Vector3f> mlCamPositions;

    std::unordered_map<int,std::vector<cv::Point2f> > mmFeatTrackingHistory;
    std::vector<int> mvFeatIDsToTrack;
    std::vector<cv::Point2f> mvFeatPtsToTrack;

    std::vector<int> mvFeatIDsInactive;
    std::vector<int> mvFeatIDsLoseTrack;

    Eigen::MatrixXf PointsForRansac;
    std::vector<cv::Point2f> mvFeatCandidates;

    Ransac* mpRansac;
    FeatureDetector* mpFeatureDetector;

    // Interact with rviz
    ros::NodeHandle mTrackerNode;
    ros::Publisher mTrackPub;
    ros::Publisher mNewerPub;

    bool mbShowTrack;
    bool mbShowNewer;

    // ---- RVIO2-SP switches ----
    // Tracker.Method: 0 = KLT (classical), 1 = SuperPoint guided keypoint matching,
    //                 2 = SuperPoint dense descriptor search + LK alignment
    int mnTrackMethod;
    // Tracker.Detector: 0 = Shi-Tomasi (classical), 1 = SuperPoint keypoints
    int mnDetector;
    // Tracker.KltGyroPrediction: seed KLT with the gyro-predicted location (method 0 only)
    bool mbKltGyroPrediction;

    bool mbSpFeedRawImage;
    bool mbSpSubPix;
    float mnSpDetThreshold;
    float mnMinDist;

    bool mbGyroPrediction;
    float mnSearchRadius;
    float mnMaxDescDist2;
    float mnRatio;
    bool mbUpdateDescriptor;
    bool mbKltFallback;
    float mnFallbackMaxDescDist2;
    int mnRefine;
    int mnRefineWin;
    float mnRefineMaxShift;

    int mnDenseRadius;
    int mnDenseStep;
    float mnDenseMinSim;
    float mnDensePeakRatio;
    float mnDenseExclusion;
    int mnDenseLkWin;
    int mnDenseLkLevel;
    float mnDenseMaxShift;

    std::unique_ptr<SuperPoint> mpSuperPoint;
    int mnDescDim;

    // Descriptors parallel to mvFeatPtsToTrack (mnDescDim per feature)
    std::vector<float> mvFeatDescToTrack;

    // ---- Statistics ----
    int mnLogEvery;
    std::ofstream mStatsCsv;
    std::unordered_map<int,int> mmFeatBirth;

    int mnStatPrev = 0;
    int mnStatDesc = 0;
    int mnStatFallback = 0;
    int mnStatRansacIn = 0;
    int mnStatNew = 0;
    double mnStatTrackMs = 0;
    double mnStatSpMs = 0;

    long mnAccFrames = 0;
    long mnAccPrev = 0;
    long mnAccDesc = 0;
    long mnAccFallback = 0;
    long mnAccRansacIn = 0;
    long mnAccNew = 0;
    long mnAccLost = 0;
    long mnAccLifetime = 0;
    double mnAccTrackMs = 0;
    double mnAccSpMs = 0;
};

} // namespace RVIO2SP

#endif
