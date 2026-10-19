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

#include <opencv2/opencv.hpp>
#include <chrono>
#include <numeric>
#include <stdexcept>
#include <tuple>
#include <ros/package.h>
#include <sensor_msgs/Image.h>

#include "Tracker.h"
#include "../util/numerics.h"


namespace RVIO2SP
{

int featId = 0;

auto red = CV_RGB(255,64,64);
auto blue = CV_RGB(64,64,255);
auto green = CV_RGB(64,255,64);
auto yellow = CV_RGB(255,200,0);

namespace
{

template <typename T>
T ReadOr(const cv::FileStorage& fs, const std::string& key, const T& def)
{
    const cv::FileNode node = fs[key];
    if (node.empty())
        return def;
    T value;
    node >> value;
    return value;
}

double MsSince(const std::chrono::steady_clock::time_point& tic)
{
    return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-tic).count();
}

} // namespace


Tracker::Tracker(const cv::FileStorage& fsSettings)
{
    const int bIsRGB = fsSettings["Camera.RGB"];
    mbIsRGB = bIsRGB;

    const int bIsFisheye = fsSettings["Camera.Fisheye"];
    mbIsFisheye = bIsFisheye;

    const int bEnableFilter = fsSettings["Tracker.EnableFilter"];
    mbEnableFilter = bEnableFilter;

    const int bEnableEqualizer = fsSettings["Tracker.EnableEqualizer"];
    mbEnableEqualizer = bEnableEqualizer;

    mnMaxFeatsPerImage = fsSettings["Tracker.nFeatures"];

    mnMaxTrackingLength = fsSettings["Tracker.nMaxTrackingLength"];
    mnMinTrackingLength = fsSettings["Tracker.nMinTrackingLength"];

    const int nMaxSlamPoints = fsSettings["Tracker.nMaxSlamPoints"];
    mbEnableSlam = nMaxSlamPoints>0 ? true : false;

    mnGoodParallax = fsSettings["Tracker.nGoodParallax"];

    mLastImage = cv::Mat();

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fsSettings["Camera.fx"];
    K.at<float>(1,1) = fsSettings["Camera.fy"];
    K.at<float>(0,2) = fsSettings["Camera.cx"];
    K.at<float>(1,2) = fsSettings["Camera.cy"];
    K.copyTo(mK);

    cv::Mat D(4,1,CV_32F);
    D.at<float>(0) = fsSettings["Camera.k1"];
    D.at<float>(1) = fsSettings["Camera.k2"];
    D.at<float>(2) = fsSettings["Camera.p1"];
    D.at<float>(3) = fsSettings["Camera.p2"];
    const float k3 = fsSettings["Camera.k3"];
    if (k3!=0)
    {
        D.resize(5);
        D.at<float>(4) = k3;
    }
    D.copyTo(mD);

    mbRestartVT = false;
    mbRefreshVT = false;

    mpRansac = new Ransac(fsSettings);
    mpFeatureDetector = new FeatureDetector(fsSettings);

    mTrackPub = mTrackerNode.advertise<sensor_msgs::Image>("/rvio2/track", 1);
    mNewerPub = mTrackerNode.advertise<sensor_msgs::Image>("/rvio2/newer", 1);

    const int bShowTrack = fsSettings["Displayer.ShowTrack"];
    mbShowTrack = bShowTrack;

    const int bShowNewer = fsSettings["Displayer.ShowNewer"];
    mbShowNewer = bShowNewer;

    // RVIO2-SP switches (defaults reproduce classical RVIO2)
    mnTrackMethod = ReadOr<int>(fsSettings, "Tracker.Method", 0);
    mnDetector = ReadOr<int>(fsSettings, "Tracker.Detector", 0);
    mbKltGyroPrediction = ReadOr<int>(fsSettings, "Tracker.KltGyroPrediction", 0);
    mnMinDist = fsSettings["Tracker.nMinDist"];

    mbSpFeedRawImage = ReadOr<int>(fsSettings, "SuperPoint.FeedRawImage", 1);
    mbSpSubPix = ReadOr<int>(fsSettings, "SuperPoint.SubPix", 1);
    mnSpDetThreshold = ReadOr<float>(fsSettings, "SuperPoint.DetThreshold", 0.005f);

    mbGyroPrediction = ReadOr<int>(fsSettings, "SPTrack.GyroPrediction", 1);
    mnSearchRadius = ReadOr<float>(fsSettings, "SPTrack.SearchRadius", 24.f);
    const float nMaxDescDist = ReadOr<float>(fsSettings, "SPTrack.MaxDescDist", 0.7f);
    mnMaxDescDist2 = nMaxDescDist*nMaxDescDist;
    mnRatio = ReadOr<float>(fsSettings, "SPTrack.Ratio", 0.9f);
    mbUpdateDescriptor = ReadOr<int>(fsSettings, "SPTrack.UpdateDescriptor", 1);
    mbKltFallback = ReadOr<int>(fsSettings, "SPTrack.KltFallback", 1);
    const float nFallbackMaxDescDist = ReadOr<float>(fsSettings, "SPTrack.FallbackMaxDescDist", 1.0f);
    mnFallbackMaxDescDist2 = nFallbackMaxDescDist*nFallbackMaxDescDist;
    mnRefine = ReadOr<int>(fsSettings, "SPTrack.Refine", 1);
    mnRefineWin = ReadOr<int>(fsSettings, "SPTrack.RefineWin", 11);
    mnRefineMaxShift = ReadOr<float>(fsSettings, "SPTrack.RefineMaxShift", 2.f);

    mnDenseRadius = ReadOr<int>(fsSettings, "SPDense.SearchRadius", 24);
    mnDenseStep = std::max(ReadOr<int>(fsSettings, "SPDense.SearchStep", 2), 1);
    mnDenseRadius = mnDenseRadius/mnDenseStep*mnDenseStep;
    mnDenseMinSim = ReadOr<float>(fsSettings, "SPDense.MinSim", 0.75f);
    mnDensePeakRatio = ReadOr<float>(fsSettings, "SPDense.PeakRatio", 0.9f);
    mnDenseExclusion = ReadOr<float>(fsSettings, "SPDense.Exclusion", 8.f);
    mnDenseLkWin = ReadOr<int>(fsSettings, "SPDense.LkWin", 15);
    mnDenseLkLevel = ReadOr<int>(fsSettings, "SPDense.LkLevel", 1);
    mnDenseMaxShift = ReadOr<float>(fsSettings, "SPDense.MaxShift", 6.f);

    if (mnTrackMethod<0 || mnTrackMethod>2)
        throw std::runtime_error("Tracker.Method must be 0 (KLT), 1 (SuperPoint guided) or 2 (SuperPoint dense)");
    if (mnDetector!=0 && mnDetector!=1)
        throw std::runtime_error("Tracker.Detector must be 0 (Shi-Tomasi) or 1 (SuperPoint)");

    mnDescDim = 0;
    if (mnTrackMethod!=0 || mnDetector==1)
    {
        mpSuperPoint.reset(new SuperPoint(fsSettings));
        mnDescDim = mpSuperPoint->DescriptorDim();
    }

    mnLogEvery = ReadOr<int>(fsSettings, "Stats.LogEvery", 30);
    const std::string strStatsCsv = ReadOr<std::string>(fsSettings, "Stats.CsvPath", "");
    if (!strStatsCsv.empty())
    {
        mStatsCsv.open(strStatsCsv, std::ofstream::out);
        mStatsCsv << "image_id,n_prev,n_desc,n_klt,n_ransac_inliers,n_new,n_tracked,sp_ms,assoc_ms\n";
    }

    std::cout << "RVIO2-SP tracker: method=" << (mnTrackMethod==2 ? "superpoint_dense" : mnTrackMethod==1 ? "superpoint" : "klt")
              << " detector=" << (mnDetector==1 ? "superpoint" : "shi-tomasi")
              << " gyro_prediction=" << (mnTrackMethod!=0 ? mbGyroPrediction : mbKltGyroPrediction)
              << " klt_fallback=" << (mnTrackMethod==1 && mbKltFallback)
              << " refine=" << (mnTrackMethod==1 ? mnRefine : 0)
              << " gpu=" << (mpSuperPoint && mpSuperPoint->UsingGpu()) << "\n";
}


Tracker::~Tracker()
{
    delete mpRansac;
    delete mpFeatureDetector;
}


void Tracker::preprocess(const int nImageId,
                         const cv::Mat& image,
                         const Eigen::Matrix3f& RcG,
                         const Eigen::Vector3f& tcG)
{
    // Convert to grayscale
    if (image.channels()==3)
    {
        if (mbIsRGB)
            cvtColor(image, image, CV_RGB2GRAY);
        else
            cvtColor(image, image, CV_BGR2GRAY);
    }
    else if (image.channels()==4)
    {
        if (mbIsRGB)
            cvtColor(image, image, CV_RGBA2GRAY);
        else
            cvtColor(image, image, CV_BGRA2GRAY);
    }

    if (mpSuperPoint && mbSpFeedRawImage)
        mpSuperPoint->Compute(image);

    if (mbEnableFilter)
    {
        cv::GaussianBlur(image, image, cv::Size(5,5), 0);
        cv::adaptiveThreshold(image, image, 225, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 5, 0);
        cv::boxFilter(image, image, image.depth(), cv::Size(5,5));
    }

    if (mbEnableEqualizer)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(5,5));
        clahe->apply(image, image);
    }

    if (mpSuperPoint && !mbSpFeedRawImage)
        mpSuperPoint->Compute(image);

    if ((int)mlCamOrientations.size()+1>mnMaxTrackingLength)
    {
        mlCamOrientations.pop_front();
        mlCamPositions.pop_front();
    }

    if (nImageId>0)
    {
        mRx = RcG*(mlCamOrientations.front().transpose());
        mRr = RcG*(mlCamOrientations.back().transpose());
    }

    mlCamOrientations.push_back(RcG);
    mlCamPositions.push_back(tcG);
}


void Tracker::undistort(const std::vector<cv::Point2f>& src,
                        std::vector<cv::Point2f>& dst)
{
    int N = src.size();

    cv::Mat mat(N,2,CV_32F);
    for(int i=0; i<N; ++i)
    {
        mat.at<float>(i,0) = src.at(i).x;
        mat.at<float>(i,1) = src.at(i).y;
    }

    mat = mat.reshape(2);

    if (!mbIsFisheye)
        cv::undistortPoints(mat, mat, mK, mD);
    else
        cv::fisheye::undistortPoints(mat, mat, mK, mD);

    mat = mat.reshape(1);

    dst.resize(N);
    for(int i=0; i<N; ++i)
    {
        dst.at(i).x = mat.at<float>(i,0);
        dst.at(i).y = mat.at<float>(i,1);
    }
}


void Tracker::DisplayTrack(const int nImageId,
                           const cv::Mat& image,
                           const std::vector<cv::Point2f>& vPrevFeatUVs,
                           const std::vector<cv::Point2f>& vCurrFeatUVs,
                           const std::vector<unsigned char>& vInlierFlags,
                           const std::vector<unsigned char>& vSources,
                           cv_bridge::CvImage& imOut)
{
    imOut.encoding = "bgr8";
    cvtColor(image, imOut.image, CV_GRAY2BGR);

    for (int i=0; i<(int)vPrevFeatUVs.size(); ++i)
    {
        if (vInlierFlags.at(i))
        {
            // Blue: KLT or descriptor match, yellow: KLT fallback of the SuperPoint tracker
            const auto color = (i<(int)vSources.size() && vSources.at(i)==2) ? yellow : blue;
            cv::circle(imOut.image, vPrevFeatUVs.at(i), 3, color, -1);
            cv::line(imOut.image, vPrevFeatUVs.at(i), vCurrFeatUVs.at(i), color);
        }
        else
            cv::circle(imOut.image, vPrevFeatUVs.at(i), 3, red, 0);
    }

    cv::putText(imOut.image, std::to_string(nImageId), cv::Point2f(15,30), cv::FONT_HERSHEY_PLAIN, 2, green, 2);
}


void Tracker::DisplayNewer(const int nImageId,
                           const cv::Mat& image,
                           const std::vector<cv::Point2f>& vRefFeatUVs,
                           const std::vector<cv::Point2f>& vNewFeatUVs,
                           cv_bridge::CvImage& imOut)
{
    imOut.encoding = "bgr8";
    cvtColor(image, imOut.image, CV_GRAY2BGR);

    for (const cv::Point2f& pt : vRefFeatUVs)
        cv::circle(imOut.image, pt, 3, blue, 0);

    for (const cv::Point2f& pt : vNewFeatUVs)
        cv::circle(imOut.image, pt, 3, green, -1);

    cv::putText(imOut.image, std::to_string(nImageId), cv::Point2f(15,30), cv::FONT_HERSHEY_PLAIN, 2, green, 2);
}


void Tracker::VisualTracking(const int nImageId,
                             const cv::Mat image,
                             int nMapPtsNeeded,
                             std::unordered_map<int,Feature*>& mFeatures)
{
    std::vector<cv::Point2f> vFeatPts, vFeatPtsUN;
    std::vector<unsigned char> vInlierFlags;
    std::vector<unsigned char> vSources;

    mnStatPrev = mvFeatPtsToTrack.size();
    mnStatDesc = 0;
    mnStatFallback = 0;
    mnStatNew = 0;
    mnStatSpMs = mpSuperPoint ? mpSuperPoint->LastInferenceMs() : 0;

    const auto tic = std::chrono::steady_clock::now();
    if (mnTrackMethod==2)
        TrackSuperPointDense(image, vFeatPts, vInlierFlags, vSources);
    else if (mnTrackMethod==1)
        TrackSuperPoint(image, vFeatPts, vInlierFlags, vSources);
    else
        TrackKLT(image, vFeatPts, vInlierFlags, vSources);
    mnStatTrackMs = MsSince(tic);

    int nFeats = vFeatPts.size();
    int nInliers = 0;

    if (std::accumulate(vInlierFlags.begin(), vInlierFlags.end(), 0)>0)
    {
        undistort(vFeatPts, vFeatPtsUN);
        Eigen::MatrixXf MatchesForRansac(3,nFeats);

        for (int i=0; i<nFeats; ++i)
        {
            cv::Point2f pt = vFeatPtsUN.at(i);
            MatchesForRansac.col(i) << pt.x, pt.y, 1;
        }

        mpRansac->FindInliers(PointsForRansac, MatchesForRansac, mRr, nInliers, vInlierFlags);

        if (nInliers==0)
        {
            std::cerr << "Visual Tracking: lost all features, refresh if the refill fails!" << "\n";
            mbRefreshVT = true;
        }
    }
    else
    {
        std::cerr << "Visual Tracking: lost all features, refresh anyway!" << "\n";
        mbRefreshVT = true;
    }

    if (mbShowTrack)
    {
        // Show the result in rviz
        cv_bridge::CvImage imTrack;
        DisplayTrack(nImageId, image, mvFeatPtsToTrack, vFeatPts, vInlierFlags, vSources, imTrack);
        mTrackPub.publish(imTrack.toImageMsg());
    }

    mnStatRansacIn = nInliers;

    std::vector<int> vFeatIDs(mvFeatIDsToTrack);
    mvFeatIDsToTrack.clear();
    mvFeatPtsToTrack.clear();

    const bool bUseDescriptors = mnTrackMethod!=0;
    std::vector<float> vPrevDescs;
    vPrevDescs.swap(mvFeatDescToTrack);

    PointsForRansac.resize(3,nInliers);

    int nFeatCnt = 0;

    for (int i=0; i<nFeats; ++i)
    {
        int id = vFeatIDs.at(i);
        Feature* pFeature = mFeatures.at(id);

        if (vInlierFlags.at(i))
        {
            cv::Point2f pt = vFeatPts.at(i);
            cv::Point2f ptUN = vFeatPtsUN.at(i);

            PointsForRansac.col(nFeatCnt) << ptUN.x, ptUN.y, 1;
            PointsForRansac.col(nFeatCnt).normalize();

            if (!pFeature->IsInited())
            {
                std::vector<cv::Point2f> vTrack;

                vTrack.swap(mmFeatTrackingHistory.at(id));
                vTrack.push_back(ptUN);

                int nTrackingLength = vTrack.size();

                if (nTrackingLength==mnMaxTrackingLength)
                {
                    if (mbEnableSlam)
                    {
                        float parallax = Parallax(vTrack.front(), vTrack.back());

                        if (nMapPtsNeeded>0)
                        {
                            if (parallax>=mnGoodParallax)
                            {
                                mvFeatInfoForInitSlam.emplace_back(id,INIT_SLAM);
                                mvvFeatMeasForInitSlam.emplace_back(vTrack);

                                nMapPtsNeeded--;
                            }
                            else
                            {
                                mvFeatInfoForPoseOnly.emplace_back(id,POSE_ONLY_M);
                                mvvFeatMeasForPoseOnly.emplace_back(vTrack);
                            }
                        }
                        else
                        {
                            if (parallax<mnGoodParallax)
                            {
                                auto vbeg = vTrack.begin()+1;
                                auto vend = vTrack.end();
                                std::vector<cv::Point2f>(vbeg,vend).swap(vTrack);
                                pFeature->reset(pFeature->RootImageId()+1);
                            }
                            else
                            {
                                mvFeatInfoForPoseOnly.emplace_back(id,POSE_ONLY_M);
                                mvvFeatMeasForPoseOnly.emplace_back(vTrack);
                            }
                        }
                    }
                    else
                    {
                        mvFeatInfoForPoseOnly.emplace_back(id,POSE_ONLY_M);
                        mvvFeatMeasForPoseOnly.emplace_back(vTrack);
                    }
                }

                mmFeatTrackingHistory.at(id).swap(vTrack);
            }
            else
            {
                if (!pFeature->IsMarginalized())
                    mvFeatMeasForExploration.emplace_back(id,ptUN);
                else
                    exit(-1);
            }

            mvFeatIDsToTrack.push_back(id);
            mvFeatPtsToTrack.push_back(pt);

            if (bUseDescriptors)
            {
                if (mbUpdateDescriptor)
                    AppendDescriptor(pt, mvFeatDescToTrack);
                else
                    mvFeatDescToTrack.insert(mvFeatDescToTrack.end(),
                                             vPrevDescs.begin()+i*mnDescDim, vPrevDescs.begin()+(i+1)*mnDescDim);
            }

            nFeatCnt++;
        }
        else
        {
            if (mmFeatBirth.count(id))
            {
                mnAccLost++;
                mnAccLifetime += nImageId-mmFeatBirth.at(id);
            }

            if (mmFeatTrackingHistory.count(id))
            {
                int nTrackingLength = mmFeatTrackingHistory.at(id).size();

                if (nTrackingLength>=mnMinTrackingLength)
                {
                    mvFeatInfoForPoseOnly.emplace_back(id,POSE_ONLY);
                    mvvFeatMeasForPoseOnly.emplace_back(mmFeatTrackingHistory.at(id));
                }

                mvFeatIDsLoseTrack.push_back(id);
            }
        }
    }

    if (!mvFeatCandidates.empty())
    {
        std::vector<cv::Point2f> vNewFeatPts;
        int nNewFeats = mpFeatureDetector->FindNewer(mvFeatCandidates, mvFeatPtsToTrack, vNewFeatPts);

        if (nNewFeats>0)
        {
            std::vector<cv::Point2f> vNewFeatPtsUN;
            undistort(vNewFeatPts, vNewFeatPtsUN);

            PointsForRansac.conservativeResize(3,nInliers+nNewFeats);

            for (int i=0; i<nNewFeats; ++i)
            {
                cv::Point2f pt = vNewFeatPts.at(i);
                cv::Point2f ptUN = vNewFeatPtsUN.at(i);

                PointsForRansac.col(nFeatCnt) << ptUN.x, ptUN.y, 1;
                PointsForRansac.col(nFeatCnt).normalize();

                int id = 0;

                if (!mvFeatIDsInactive.empty())
                {
                    id = mvFeatIDsInactive.back();
                    mFeatures.at(id)->reset(nImageId);
                    mvFeatIDsInactive.pop_back();
                }
                else
                {
                    id = featId++;
                    mFeatures[id] = new Feature(id, nImageId);
                }

                mmFeatTrackingHistory[id].reserve(mnMaxTrackingLength);
                mmFeatTrackingHistory[id].push_back(ptUN);

                mvFeatIDsToTrack.push_back(id);
                mvFeatPtsToTrack.push_back(pt);
                mmFeatBirth[id] = nImageId;

                if (bUseDescriptors)
                    AppendDescriptor(pt, mvFeatDescToTrack);

                nFeatCnt++;
            }

            mnStatNew = nNewFeats;

            if (mbShowNewer)
            {
                // Show the result in rviz
                cv_bridge::CvImage imNewer;
                DisplayNewer(nImageId, image, mvFeatPtsToTrack, vNewFeatPts, imNewer);
                mNewerPub.publish(imNewer.toImageMsg());
            }

            if (mbRefreshVT)
                mbRefreshVT = false;
        }
    }

    image.copyTo(mLastImage);

    LogStats(nImageId);
}


bool Tracker::start(const int nImageId,
                    const cv::Mat& image,
                    const Eigen::Matrix3f& RcG,
                    const Eigen::Vector3f& tcG,
                    std::unordered_map<int,Feature*>& mFeatures)
{
    if (nImageId==0 || mbRestartVT)
    {
        featId = 0;
        mFeatures.clear();
    }

    mvFeatIDsToTrack.clear();
    mvFeatPtsToTrack.clear();
    mvFeatDescToTrack.clear();
    mmFeatTrackingHistory.clear();

    preprocess(nImageId, image, RcG, tcG);

    int nFeats = Detect(image, mnMaxFeatsPerImage, mvFeatPtsToTrack);
    if (nFeats==0)
    {
        std::cerr << "No features available to track!" << "\n";

        if (!mbRestartVT && !mbRefreshVT)
            mbRestartVT = true;

        return false;
    }

    if (mbRestartVT)
        mbRestartVT = false;

    if (mbRefreshVT)
    {
        mlCamOrientations.clear();
        mlCamPositions.clear();

        mvFeatInfoForInitSlam.clear();
        mvvFeatMeasForInitSlam.clear();
        mvFeatInfoForPoseOnly.clear();
        mvvFeatMeasForPoseOnly.clear();
        mvFeatMeasForExploration.clear();

        mbRefreshVT = false;
    }

    std::vector<cv::Point2f> vFeatPtsToTrackUN;
    undistort(mvFeatPtsToTrack, vFeatPtsToTrackUN);

    PointsForRansac.resize(3,nFeats);

    for (int i=0; i<nFeats; ++i)
    {
        cv::Point2f ptUN = vFeatPtsToTrackUN.at(i);

        PointsForRansac.col(i) << ptUN.x, ptUN.y, 1;
        PointsForRansac.col(i).normalize();

        int id = featId++;
        mFeatures[id] = new Feature(id, nImageId);

        mmFeatTrackingHistory[id].reserve(mnMaxTrackingLength);
        mmFeatTrackingHistory[id].push_back(ptUN);

        mvFeatIDsToTrack.push_back(id);
        mmFeatBirth[id] = nImageId;
    }

    mvFeatDescToTrack.clear();
    if (mnTrackMethod!=0)
    {
        for (const cv::Point2f& pt : mvFeatPtsToTrack)
            AppendDescriptor(pt, mvFeatDescToTrack);
    }

    image.copyTo(mLastImage);

    return true;
}


void Tracker::manage(const int nImageId,
                     const cv::Mat& image,
                     const Eigen::Matrix3f& RcG,
                     const Eigen::Vector3f& tcG,
                     const std::unordered_map<int,Feature*>& mFeatures)
{
    if (!mvFeatInfoForInitSlam.empty())
    {
        for (const std::pair<int,Type>& vit : mvFeatInfoForInitSlam)
        {
            int id = vit.first;
            int type = vit.second;

            Feature* pFeature = mFeatures.at(id);

            if (!pFeature->IsInited())
            {
                int N = 0;
                if (type==UNUSED)
                    N = 1;
                else if (type==BAD)
                    N = mnMinTrackingLength;
                else if (type==POSE_ONLY_M)
                    N = mnMaxTrackingLength-1;
                else
                    exit(-1);

                auto vbeg = mmFeatTrackingHistory.at(id).begin()+N;
                auto vend = mmFeatTrackingHistory.at(id).end();
                std::vector<cv::Point2f>(vbeg,vend).swap(mmFeatTrackingHistory.at(id));
                pFeature->reset(pFeature->RootImageId()+N);
            }
            else
                mmFeatTrackingHistory.erase(id);
        }
    }

    if (!mvFeatInfoForPoseOnly.empty())
    {
        for (const std::pair<int,Type>& vit : mvFeatInfoForPoseOnly)
        {
            int id = vit.first;
            int type = vit.second;

            Feature* pFeature = mFeatures.at(id);

            if (type!=POSE_ONLY)
            {
                int N = 0;
                if (type==UNUSED)
                    N = 1;
                else if (type==BAD)
                    N = mnMinTrackingLength;
                else if (type==POSE_ONLY_M)
                    N = mnMaxTrackingLength-1;
                else
                    exit(-1);

                auto vbeg = mmFeatTrackingHistory.at(id).begin()+N;
                auto vend = mmFeatTrackingHistory.at(id).end();
                std::vector<cv::Point2f>(vbeg,vend).swap(mmFeatTrackingHistory.at(id));
                pFeature->reset(pFeature->RootImageId()+N);
            }
        }
    }

    if (!mvFeatIDsLoseTrack.empty())
    {
        for (const int& id : mvFeatIDsLoseTrack)
        {
            mFeatures.at(id)->clear();
            mvFeatIDsInactive.push_back(id);
            mmFeatTrackingHistory.at(id).clear();
        }

        mvFeatIDsLoseTrack.clear();
    }

    mvFeatInfoForInitSlam.clear();
    mvvFeatMeasForInitSlam.clear();
    mvFeatInfoForPoseOnly.clear();
    mvvFeatMeasForPoseOnly.clear();
    mvFeatMeasForExploration.clear();

    preprocess(nImageId, image, RcG, tcG);

    Detect(image, 1.5*mnMaxFeatsPerImage, mvFeatCandidates);
}


void Tracker::track(const int nImageId,
                    const cv::Mat& image,
                    const Eigen::Matrix3f& RcG,
                    const Eigen::Vector3f& tcG,
                    int nMapPtsNeeded,
                    std::unordered_map<int,Feature*>& mFeatures)
{
    if (nImageId==0 || mbRestartVT || mbRefreshVT)
    {
        if (!start(nImageId, image, RcG, tcG, mFeatures))
            return;
    }
    else
    {
        manage(nImageId, image, RcG, tcG, mFeatures);

        VisualTracking(nImageId, image, nMapPtsNeeded, mFeatures);
    }
}

int Tracker::Detect(const cv::Mat& image,
                    const int nCorners,
                    std::vector<cv::Point2f>& vCorners)
{
    if (mnDetector!=1)
        return mpFeatureDetector->DetectWithSubPix(image, nCorners, 1, vCorners);

    vCorners.clear();
    vCorners.reserve(nCorners);

    // Strongest SuperPoint keypoints with the same minimum spacing as goodFeaturesToTrack
    const float cellSize = std::max(mnMinDist, 1.f);
    const int nGridCols = image.cols/cellSize+1;
    const int nGridRows = image.rows/cellSize+1;
    std::vector<std::vector<cv::Point2f> > vvGrid(nGridCols*nGridRows);
    const float minDist2 = mnMinDist*mnMinDist;

    for (const cv::KeyPoint& kp : mpSuperPoint->Keypoints())
    {
        if (kp.response<mnSpDetThreshold || (int)vCorners.size()>=nCorners)
            break;

        const int col = std::min((int)(kp.pt.x/cellSize), nGridCols-1);
        const int row = std::min((int)(kp.pt.y/cellSize), nGridRows-1);

        bool bFree = true;
        for (int r=std::max(row-1,0); bFree && r<=std::min(row+1,nGridRows-1); ++r)
        {
            for (int c=std::max(col-1,0); bFree && c<=std::min(col+1,nGridCols-1); ++c)
            {
                for (const cv::Point2f& q : vvGrid.at(r*nGridCols+c))
                {
                    const cv::Point2f d = kp.pt-q;
                    if (d.dot(d)<minDist2)
                    {
                        bFree = false;
                        break;
                    }
                }
            }
        }

        if (bFree)
        {
            vvGrid.at(row*nGridCols+col).push_back(kp.pt);
            vCorners.push_back(kp.pt);
        }
    }

    if (mbSpSubPix && !vCorners.empty())
    {
        cv::Size subPixWinSize(std::floor(.5*mnMinDist),std::floor(.5*mnMinDist));
        cv::TermCriteria subPixCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 1e-2);
        cv::cornerSubPix(image, vCorners, subPixWinSize, cv::Size(-1,-1), subPixCriteria);
    }

    return (int)vCorners.size();
}


void Tracker::TrackKLT(const cv::Mat& image,
                       std::vector<cv::Point2f>& vFeatPts,
                       std::vector<unsigned char>& vInlierFlags,
                       std::vector<unsigned char>& vSources)
{
    std::vector<float> vErrors;

    cv::Size winSize(15,15);
    cv::TermCriteria termCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 1e-2);

    int flags = cv::OPTFLOW_LK_GET_MIN_EIGENVALS;
    if (mbKltGyroPrediction && !mvFeatPtsToTrack.empty())
    {
        PredictWithGyro(mvFeatPtsToTrack, vFeatPts);
        flags |= cv::OPTFLOW_USE_INITIAL_FLOW;
    }

    // LK method
    cv::calcOpticalFlowPyrLK(mLastImage, image, mvFeatPtsToTrack, vFeatPts, vInlierFlags, vErrors,
                             winSize, 3, termCriteria, flags, 1e-4);

    vSources.assign(vInlierFlags.size(), 0);
    mnStatFallback = std::accumulate(vInlierFlags.begin(), vInlierFlags.end(), 0);
}


void Tracker::TrackSuperPoint(const cv::Mat& image,
                              std::vector<cv::Point2f>& vFeatPts,
                              std::vector<unsigned char>& vInlierFlags,
                              std::vector<unsigned char>& vSources)
{
    const int N = mvFeatPtsToTrack.size();
    const int D = mnDescDim;

    vInlierFlags.assign(N, 0);
    vSources.assign(N, 0);

    if (N==0)
    {
        vFeatPts.clear();
        return;
    }

    if ((int)mvFeatDescToTrack.size()!=N*D)
    {
        ROS_WARN("RVIO2-SP: descriptor/track mismatch (%zu vs %d), using KLT for this frame",
                 mvFeatDescToTrack.size(), N*D);
        TrackKLT(image, vFeatPts, vInlierFlags, vSources);
        mvFeatDescToTrack.clear();
        for (const cv::Point2f& pt : mvFeatPtsToTrack)
            AppendDescriptor(pt, mvFeatDescToTrack);
        return;
    }

    std::vector<cv::Point2f> vPredPts;
    if (mbGyroPrediction)
        PredictWithGyro(mvFeatPtsToTrack, vPredPts);
    else
        vPredPts = mvFeatPtsToTrack;

    vFeatPts = vPredPts;

    const std::vector<cv::KeyPoint>& vKeypoints = mpSuperPoint->Keypoints();
    const int M = vKeypoints.size();

    // Spatial hash of the current keypoints
    const float cellSize = std::max(mnSearchRadius, 1.f);
    const int nGridCols = image.cols/cellSize+1;
    const int nGridRows = image.rows/cellSize+1;
    std::vector<std::vector<int> > vvGrid(nGridCols*nGridRows);
    for (int j=0; j<M; ++j)
    {
        const cv::Point2f& pt = vKeypoints.at(j).pt;
        const int col = std::min(std::max((int)(pt.x/cellSize),0), nGridCols-1);
        const int row = std::min(std::max((int)(pt.y/cellSize),0), nGridRows-1);
        vvGrid.at(row*nGridCols+col).push_back(j);
    }

    // Keypoint descriptors are sampled lazily
    std::vector<float> vKpDescs(M*D);
    std::vector<unsigned char> vKpHasDesc(M, 0);

    const float radius2 = mnSearchRadius*mnSearchRadius;
    const float ratio2 = mnRatio*mnRatio;

    // Candidate (distance, track, keypoint) pairs that pass the distance and ratio tests
    std::vector<std::tuple<float,int,int> > vCandidates;
    vCandidates.reserve(N);

    for (int i=0; i<N; ++i)
    {
        const cv::Point2f& pred = vPredPts.at(i);
        const float* trackDesc = &mvFeatDescToTrack[i*D];

        const int col = (int)std::floor(pred.x/cellSize);
        const int row = (int)std::floor(pred.y/cellSize);

        float best = 1e9f, second = 1e9f;
        int bestIdx = -1;

        for (int r=std::max(row-1,0); r<=std::min(row+1,nGridRows-1); ++r)
        {
            for (int c=std::max(col-1,0); c<=std::min(col+1,nGridCols-1); ++c)
            {
                for (const int j : vvGrid.at(r*nGridCols+c))
                {
                    const cv::Point2f d = vKeypoints.at(j).pt-pred;
                    if (d.dot(d)>radius2)
                        continue;

                    if (!vKpHasDesc.at(j))
                    {
                        mpSuperPoint->Describe(vKeypoints.at(j).pt, &vKpDescs[j*D]);
                        vKpHasDesc.at(j) = 1;
                    }

                    const float dist2 = DescDist2(trackDesc, &vKpDescs[j*D], D);
                    if (dist2<best)
                    {
                        second = best;
                        best = dist2;
                        bestIdx = j;
                    }
                    else if (dist2<second)
                        second = dist2;
                }
            }
        }

        if (bestIdx>=0 && best<=mnMaxDescDist2 && (second>=1e9f || best<=ratio2*second))
            vCandidates.emplace_back(best, i, bestIdx);
    }

    // One-to-one assignment, best matches first
    std::sort(vCandidates.begin(), vCandidates.end());
    std::vector<unsigned char> vKpUsed(M, 0);
    std::vector<int> vMatched;
    vMatched.reserve(vCandidates.size());

    for (const auto& cand : vCandidates)
    {
        const int i = std::get<1>(cand);
        const int j = std::get<2>(cand);
        if (vKpUsed.at(j))
            continue;
        vKpUsed.at(j) = 1;

        vFeatPts.at(i) = vKeypoints.at(j).pt;
        vInlierFlags.at(i) = 1;
        vSources.at(i) = 1;
        vMatched.push_back(i);
    }
    mnStatDesc = vMatched.size();

    cv::TermCriteria termCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 1e-2);

    // Sub-pixel refinement of the descriptor matches
    if (mnRefine!=0 && !vMatched.empty())
    {
        std::vector<cv::Point2f> vPrev, vRefined;
        vPrev.reserve(vMatched.size());
        vRefined.reserve(vMatched.size());
        for (const int i : vMatched)
        {
            vPrev.push_back(mvFeatPtsToTrack.at(i));
            vRefined.push_back(vFeatPts.at(i));
        }

        std::vector<unsigned char> vStatus(vMatched.size(), 1);
        if (mnRefine==1)
        {
            // Single-level LK seeded at the descriptor match
            std::vector<float> vErrors;
            cv::calcOpticalFlowPyrLK(mLastImage, image, vPrev, vRefined, vStatus, vErrors,
                                     cv::Size(mnRefineWin,mnRefineWin), 0, termCriteria,
                                     cv::OPTFLOW_USE_INITIAL_FLOW|cv::OPTFLOW_LK_GET_MIN_EIGENVALS, 1e-4);
        }
        else
        {
            const int half = std::max(mnRefineWin/2, 2);
            cv::cornerSubPix(image, vRefined, cv::Size(half,half), cv::Size(-1,-1), termCriteria);
        }

        for (size_t k=0; k<vMatched.size(); ++k)
        {
            const int i = vMatched.at(k);
            const cv::Point2f d = vRefined.at(k)-vFeatPts.at(i);
            if (vStatus.at(k) && d.dot(d)<=mnRefineMaxShift*mnRefineMaxShift)
                vFeatPts.at(i) = vRefined.at(k);
        }
    }

    // KLT fallback for tracks SuperPoint did not re-detect, verified by descriptor
    if (mbKltFallback && (int)vMatched.size()<N)
    {
        std::vector<int> vUnmatched;
        std::vector<cv::Point2f> vPrev, vNext;
        for (int i=0; i<N; ++i)
        {
            if (!vInlierFlags.at(i))
            {
                vUnmatched.push_back(i);
                vPrev.push_back(mvFeatPtsToTrack.at(i));
                vNext.push_back(vPredPts.at(i));
            }
        }

        std::vector<unsigned char> vStatus;
        std::vector<float> vErrors;
        cv::calcOpticalFlowPyrLK(mLastImage, image, vPrev, vNext, vStatus, vErrors,
                                 cv::Size(15,15), 3, termCriteria,
                                 cv::OPTFLOW_USE_INITIAL_FLOW|cv::OPTFLOW_LK_GET_MIN_EIGENVALS, 1e-4);

        std::vector<float> vDesc(D);
        for (size_t k=0; k<vUnmatched.size(); ++k)
        {
            if (!vStatus.at(k))
                continue;

            const cv::Point2f& pt = vNext.at(k);
            if (pt.x<0 || pt.y<0 || pt.x>image.cols-1 || pt.y>image.rows-1)
                continue;

            const int i = vUnmatched.at(k);
            mpSuperPoint->Describe(pt, vDesc.data());
            if (DescDist2(&mvFeatDescToTrack[i*D], vDesc.data(), D)>mnFallbackMaxDescDist2)
                continue;

            vFeatPts.at(i) = pt;
            vInlierFlags.at(i) = 1;
            vSources.at(i) = 2;
            mnStatFallback++;
        }
    }
}


void Tracker::TrackSuperPointDense(const cv::Mat& image,
                                   std::vector<cv::Point2f>& vFeatPts,
                                   std::vector<unsigned char>& vInlierFlags,
                                   std::vector<unsigned char>& vSources)
{
    const int N = mvFeatPtsToTrack.size();
    const int D = mnDescDim;

    vInlierFlags.assign(N, 0);
    vSources.assign(N, 0);

    if (N==0)
    {
        vFeatPts.clear();
        return;
    }

    if ((int)mvFeatDescToTrack.size()!=N*D)
    {
        ROS_WARN("RVIO2-SP: descriptor/track mismatch (%zu vs %d), using KLT for this frame",
                 mvFeatDescToTrack.size(), N*D);
        TrackKLT(image, vFeatPts, vInlierFlags, vSources);
        mvFeatDescToTrack.clear();
        for (const cv::Point2f& pt : mvFeatPtsToTrack)
            AppendDescriptor(pt, mvFeatDescToTrack);
        return;
    }

    std::vector<cv::Point2f> vPredPts;
    if (mbGyroPrediction)
        PredictWithGyro(mvFeatPtsToTrack, vPredPts);
    else
        vPredPts = mvFeatPtsToTrack;
    vFeatPts = vPredPts;

    // 1. Correlate each track descriptor over a window of the dense descriptor field
    const int n = 2*(mnDenseRadius/mnDenseStep)+1;
    const float exclusion2 = mnDenseExclusion*mnDenseExclusion;
    std::vector<float> vSims;
    std::vector<int> vCandidates;
    std::vector<cv::Point2f> vPrev, vPeaks;

    for (int i=0; i<N; ++i)
    {
        mpSuperPoint->SimilarityWindow(&mvFeatDescToTrack[i*D], vPredPts.at(i), mnDenseRadius, mnDenseStep, vSims);

        const int best = std::max_element(vSims.begin(), vSims.end())-vSims.begin();
        const float bestSim = vSims.at(best);
        if (bestSim<mnDenseMinSim)
            continue;

        const int bx = best%n, by = best/n;
        float second = -1;
        for (int k=0; k<n*n; ++k)
        {
            const float dx = (float)((k%n-bx)*mnDenseStep), dy = (float)((k/n-by)*mnDenseStep);
            if (dx*dx+dy*dy>exclusion2)
                second = std::max(second, vSims.at(k));
        }

        // 2. Reject ambiguous windows (a distant second peak nearly as similar)
        if ((1-second)*mnDensePeakRatio<(1-bestSim))
            continue;

        vCandidates.push_back(i);
        vPrev.push_back(mvFeatPtsToTrack.at(i));
        vPeaks.push_back(vPredPts.at(i)+cv::Point2f((float)(bx*mnDenseStep-mnDenseRadius), (float)(by*mnDenseStep-mnDenseRadius)));
    }

    if (vCandidates.empty())
        return;

    // 3. Sub-pixel alignment from the previous image, seeded at the descriptor peak
    std::vector<cv::Point2f> vAligned(vPeaks);
    std::vector<unsigned char> vStatus;
    std::vector<float> vErrors;
    cv::TermCriteria termCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 1e-2);
    cv::calcOpticalFlowPyrLK(mLastImage, image, vPrev, vAligned, vStatus, vErrors,
                             cv::Size(mnDenseLkWin,mnDenseLkWin), mnDenseLkLevel, termCriteria,
                             cv::OPTFLOW_USE_INITIAL_FLOW|cv::OPTFLOW_LK_GET_MIN_EIGENVALS, 1e-4);

    // 4. Keep alignments that stay near the peak and still match the descriptor
    std::vector<float> vDesc(D);
    const float minDist2 = 2-2*mnDenseMinSim;
    for (size_t k=0; k<vCandidates.size(); ++k)
    {
        const cv::Point2f& pt = vAligned.at(k);
        const cv::Point2f shift = pt-vPeaks.at(k);
        if (!vStatus.at(k) || shift.dot(shift)>mnDenseMaxShift*mnDenseMaxShift)
            continue;
        if (pt.x<0 || pt.y<0 || pt.x>image.cols-1 || pt.y>image.rows-1)
            continue;

        const int i = vCandidates.at(k);
        mpSuperPoint->Describe(pt, vDesc.data());
        if (DescDist2(&mvFeatDescToTrack[i*D], vDesc.data(), D)>minDist2)
            continue;

        vFeatPts.at(i) = pt;
        vInlierFlags.at(i) = 1;
        vSources.at(i) = 1;
        mnStatDesc++;
    }
}


void Tracker::PredictWithGyro(const std::vector<cv::Point2f>& vPrevPts,
                              std::vector<cv::Point2f>& vPredPts)
{
    vPredPts = vPrevPts;
    if (vPrevPts.empty())
        return;

    std::vector<cv::Point2f> vPrevPtsUN;
    undistort(vPrevPts, vPrevPtsUN);

    // Rotate bearings from the previous into the current camera frame
    std::vector<cv::Point3f> vRays;
    std::vector<int> vIdx;
    vRays.reserve(vPrevPts.size());
    vIdx.reserve(vPrevPts.size());
    for (int i=0; i<(int)vPrevPtsUN.size(); ++i)
    {
        const Eigen::Vector3f ray = mRr*Eigen::Vector3f(vPrevPtsUN.at(i).x, vPrevPtsUN.at(i).y, 1);
        if (ray(2)<.1f)
            continue;
        vRays.emplace_back(ray(0), ray(1), ray(2));
        vIdx.push_back(i);
    }

    if (vRays.empty())
        return;

    std::vector<cv::Point2f> vProjected;
    const cv::Mat rvec = cv::Mat::zeros(3,1,CV_64F);
    const cv::Mat tvec = cv::Mat::zeros(3,1,CV_64F);
    if (!mbIsFisheye)
        cv::projectPoints(vRays, rvec, tvec, mK, mD, vProjected);
    else
        cv::fisheye::projectPoints(vRays, vProjected, rvec, tvec, mK, mD);

    for (size_t k=0; k<vIdx.size(); ++k)
        vPredPts.at(vIdx.at(k)) = vProjected.at(k);
}


void Tracker::AppendDescriptor(const cv::Point2f& pt,
                               std::vector<float>& vDescs)
{
    const size_t offset = vDescs.size();
    vDescs.resize(offset+mnDescDim);
    mpSuperPoint->Describe(pt, &vDescs[offset]);
}


void Tracker::LogStats(const int nImageId)
{
    if (mStatsCsv.is_open())
    {
        mStatsCsv << nImageId << "," << mnStatPrev << "," << mnStatDesc << ","
                  << mnStatFallback << ","
                  << mnStatRansacIn << "," << mnStatNew << "," << mvFeatPtsToTrack.size() << ","
                  << mnStatSpMs << "," << mnStatTrackMs << "\n";
    }

    mnAccFrames++;
    mnAccPrev += mnStatPrev;
    mnAccDesc += mnStatDesc;
    mnAccFallback += mnStatFallback;
    mnAccRansacIn += mnStatRansacIn;
    mnAccNew += mnStatNew;
    mnAccTrackMs += mnStatTrackMs;
    mnAccSpMs += mnStatSpMs;

    if (mnLogEvery<=0 || mnAccFrames<mnLogEvery)
        return;

    const double n = mnAccFrames;
    if (mnTrackMethod!=0)
    {
        ROS_INFO("RVIO2-SP [superpoint] img=%d tracks=%.0f desc=%.0f klt_fb=%.0f ransac_in=%.0f new=%.0f "
                 "lost=%ld mean_life=%.1f sp=%.1fms assoc=%.1fms",
                 nImageId, mnAccPrev/n, mnAccDesc/n, mnAccFallback/n, mnAccRansacIn/n, mnAccNew/n,
                 mnAccLost, mnAccLost ? (double)mnAccLifetime/mnAccLost : 0., mnAccSpMs/n, mnAccTrackMs/n);
    }
    else
    {
        ROS_INFO("RVIO2-SP [klt] img=%d tracks=%.0f klt_ok=%.0f ransac_in=%.0f new=%.0f lost=%ld mean_life=%.1f "
                 "sp=%.1fms assoc=%.1fms",
                 nImageId, mnAccPrev/n, mnAccFallback/n, mnAccRansacIn/n, mnAccNew/n,
                 mnAccLost, mnAccLost ? (double)mnAccLifetime/mnAccLost : 0., mnAccSpMs/n, mnAccTrackMs/n);
    }

    mnAccFrames = mnAccPrev = mnAccDesc = mnAccFallback = mnAccRansacIn = mnAccNew = 0;
    mnAccLost = mnAccLifetime = 0;
    mnAccTrackMs = mnAccSpMs = 0;
}

} // namespace RVIO2SP
