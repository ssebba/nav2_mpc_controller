#include <cmath>
#include <numbers>
#include <stdexcept>

#include <gtest/gtest.h>

#include "mpc_core/reference_builder.hpp"
#include "test_utils.hpp"

using mpc_core::Point2d;
using mpc_core::ReferenceBuilder;
using mpc_core::StateTrajectory;
using mpc_core::StateVec;
using mpc_core::test::figureEight;
using mpc_core::test::straightPath;

TEST(ReferenceBuilder, OutputSizes)
{
  const auto ref = ReferenceBuilder::build(straightPath(5.0, 0.05), StateVec::Zero(), 0.5, 0.1, 20);
  EXPECT_EQ(ref.states.size(), 21u);
  EXPECT_EQ(ref.controls.size(), 20u);
}

TEST(ReferenceBuilder, SamplesAreSpacedByVrefTimesDt)
{
  const double v_ref = 0.5;
  const double dt = 0.1;
  // Waypoint spacing deliberately not a multiple of v_ref * dt to exercise interpolation.
  const auto ref =
    ReferenceBuilder::build(straightPath(5.0, 0.07), StateVec::Zero(), v_ref, dt, 10);
  for (std::size_t k = 1; k < ref.states.size(); ++k) {
    EXPECT_NEAR(ref.states[k].x() - ref.states[k - 1].x(), v_ref * dt, 1e-9) << "k = " << k;
    EXPECT_NEAR(ref.states[k].y(), 0.0, 1e-12);
  }
}

TEST(ReferenceBuilder, FeedForwardMatchesSampling)
{
  const auto ref = ReferenceBuilder::build(straightPath(5.0, 0.05), StateVec::Zero(), 0.4, 0.1, 10);
  for (const auto & u : ref.controls) {
    EXPECT_NEAR(u.x(), 0.4, 1e-9);
    EXPECT_NEAR(u.y(), 0.0, 1e-9);
  }
}

TEST(ReferenceBuilder, StartsFromProjectionOnThePath)
{
  // Robot between two waypoints and off the path: the first sample is its projection.
  const auto ref =
    ReferenceBuilder::build(straightPath(5.0, 0.5), StateVec(1.23, 0.3, 0.0), 0.5, 0.1, 5);
  EXPECT_NEAR(ref.states.front().x(), 1.23, 1e-9);
  EXPECT_NEAR(ref.states.front().y(), 0.0, 1e-9);
}

TEST(ReferenceBuilder, HoldsLastPoseAndStopsAtEndOfPath)
{
  const auto ref = ReferenceBuilder::build(straightPath(0.3, 0.1), StateVec::Zero(), 0.5, 0.1, 20);
  EXPECT_NEAR(ref.states.back().x(), 0.3, 1e-9);
  EXPECT_NEAR(ref.controls.back().x(), 0.0, 1e-9);
}

TEST(ReferenceBuilder, YawIsUnwrappedAroundRobotYaw)
{
  // Path heading at -pi + 0.1, robot heading at pi - 0.1: the difference is 0.2 rad, not ~2*pi.
  StateTrajectory path;
  for (int i = 0; i < 50; ++i) {
    path.emplace_back(-0.1 * i, 0.0, -std::numbers::pi + 0.1);
  }
  const auto ref =
    ReferenceBuilder::build(path, StateVec(0.0, 0.0, std::numbers::pi - 0.1), 0.5, 0.1, 10);
  for (const auto & r : ref.states) {
    EXPECT_NEAR(r.z(), std::numbers::pi + 0.1, 1e-9);
  }
}

TEST(ReferenceBuilder, TangentYawKeepsGoalOrientation)
{
  StateTrajectory path{{0.0, 0.0, 0.0}, {1.0, 1.0, 0.0}, {1.0, 2.0, 0.0}, {1.0, 3.0, -1.0}};
  ReferenceBuilder::assignTangentYaw(path);
  EXPECT_NEAR(path[0].z(), std::numbers::pi / 4.0, 1e-12);
  EXPECT_NEAR(path[1].z(), std::numbers::pi / 2.0, 1e-12);
  EXPECT_NEAR(path[2].z(), std::numbers::pi / 2.0, 1e-12);
  EXPECT_NEAR(path[3].z(), -1.0, 1e-12);
}

TEST(ReferenceBuilder, SearchWindowHandlesSelfIntersectingPaths)
{
  // On a figure eight the crossing point is visited twice: with a short search window
  // the closest point must stay on the first branch.
  const auto path = figureEight(1.0, 0.02);
  const Point2d robot(0.0, 0.0);
  const auto near = ReferenceBuilder::closestPoint(path, robot, 0.5);
  EXPECT_LT(near.segment, 10u);
  const auto global = ReferenceBuilder::closestPoint(path, Point2d(0.0, 0.01), 1e9);
  EXPECT_NEAR(global.distance, 0.0, 0.02);
}

TEST(ReferenceBuilder, ThrowsOnInvalidInput)
{
  EXPECT_THROW(ReferenceBuilder::build({}, StateVec::Zero(), 0.5, 0.1, 10), std::invalid_argument);
  EXPECT_THROW(
    ReferenceBuilder::build(straightPath(1.0, 0.1), StateVec::Zero(), 0.5, 0.0, 10),
    std::invalid_argument);
}
