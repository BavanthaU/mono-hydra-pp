/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testDepthFactor.cpp
 * @brief  Tests for sparse depth factors used by the RGB-D backend.
 */

#include <gtest/gtest.h>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

#include "kimera-vio/factors/DepthFactor.h"
#include "kimera-vio/test/EvaluateFactor.h"

using namespace gtsam;
using namespace VIO;

namespace {

static constexpr double kTol = 1e-5;
static constexpr double kDelta = 1e-5;

DepthFactor makeDepthFactor(double measured_depth) {
  const Cal3_S2 calib(320.0, 320.0, 0.0, 320.0, 240.0);
  const auto noise = noiseModel::Isotropic::Sigma(1, 0.1);
  return DepthFactor(Symbol('x', 0),
                     Symbol('l', 0),
                     320.0,
                     240.0,
                     measured_depth,
                     calib,
                     Pose3(),
                     noise);
}

}  // namespace

TEST(testDepthFactor, ErrorIsZeroAtMeasuredDepth) {
  const DepthFactor factor = makeDepthFactor(2.0);

  const Pose3 body_pose;
  const Point3 point(0.0, 0.0, 2.0);
  const Vector error = factor.evaluateError(body_pose, point);

  EXPECT_TRUE(assert_equal(Vector1::Zero(), error, kTol));
}

TEST(testDepthFactor, ErrorUsesMeasuredMinusProjectedDepth) {
  const DepthFactor factor = makeDepthFactor(2.0);

  const Pose3 body_pose;
  const Point3 point(0.0, 0.0, 3.0);
  const Vector error = factor.evaluateError(body_pose, point);

  EXPECT_TRUE(assert_equal(Vector1::Constant(-1.0), error, kTol));
}

TEST(testDepthFactor, JacobiansMatchNumericalDerivative) {
  const DepthFactor factor = makeDepthFactor(2.0);

  const Pose3 body_pose(Rot3::RzRyRx(0.1, -0.2, 0.05),
                        Point3(0.3, -0.1, 0.2));
  const Point3 point(0.2, -0.3, 3.1);

  VIO::test::evaluateFactor(factor, body_pose, point, kTol, kDelta);
}

TEST(testDepthFactor, OptimizationPullsLandmarkTowardMeasuredDepth) {
  const Key pose_key = Symbol('x', 0);
  const Key point_key = Symbol('l', 0);

  const Pose3 body_pose;
  const Point3 initial_point(0.0, 0.0, 3.0);

  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Pose3>>(
      pose_key, body_pose, noiseModel::Isotropic::Sigma(6, 1e-6));
  graph.emplace_shared<PriorFactor<Point3>>(
      point_key, initial_point, noiseModel::Isotropic::Sigma(3, 10.0));

  const Cal3_S2 calib(320.0, 320.0, 0.0, 320.0, 240.0);
  graph.emplace_shared<DepthFactor>(
      pose_key,
      point_key,
      320.0,
      240.0,
      2.0,
      calib,
      Pose3(),
      noiseModel::Isotropic::Sigma(1, 0.1));

  Values initial_values;
  initial_values.insert(pose_key, body_pose);
  initial_values.insert(point_key, initial_point);

  const Values result =
      GaussNewtonOptimizer(graph, initial_values).optimize();
  const Point3 optimized_point = result.at<Point3>(point_key);

  EXPECT_LT(optimized_point.z(), 2.01);
  EXPECT_GT(optimized_point.z(), 1.99);
}

TEST(testDepthFactor, ProjectionAndDepthConstrainInitializedRgbdLandmark) {
  const Key pose_key = Symbol('x', 0);
  const Key point_key = Symbol('l', 0);

  const Pose3 body_pose;
  const Point3 initial_point(0.2, -0.2, 3.0);
  const auto calib = boost::make_shared<Cal3_S2>(
      320.0, 320.0, 0.0, 320.0, 240.0);

  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Pose3>>(
      pose_key, body_pose, noiseModel::Isotropic::Sigma(6, 1e-6));
  graph.emplace_shared<GenericProjectionFactor<Pose3, Point3>>(
      Point2(320.0, 240.0),
      noiseModel::Isotropic::Sigma(2, 1.0),
      pose_key,
      point_key,
      calib,
      true,
      true,
      Pose3());
  graph.emplace_shared<DepthFactor>(
      pose_key,
      point_key,
      320.0,
      240.0,
      2.0,
      *calib,
      Pose3(),
      noiseModel::Isotropic::Sigma(1, 0.1));

  Values initial_values;
  initial_values.insert(pose_key, body_pose);
  initial_values.insert(point_key, initial_point);

  const Values result =
      GaussNewtonOptimizer(graph, initial_values).optimize();
  const Point3 optimized_point = result.at<Point3>(point_key);

  EXPECT_NEAR(0.0, optimized_point.x(), 1e-3);
  EXPECT_NEAR(0.0, optimized_point.y(), 1e-3);
  EXPECT_NEAR(2.0, optimized_point.z(), 1e-3);
}
