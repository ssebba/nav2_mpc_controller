#include <cmath>
#include <numbers>
#include <stdexcept>

#include <gtest/gtest.h>

#include "mpc_core/obstacle_constraints.hpp"

using mpc_core::buildObstacleConstraints;
using mpc_core::downsamplePoints;
using mpc_core::minimumClearance;
using mpc_core::ObstacleConfig;
using mpc_core::ObstaclePoints;
using mpc_core::Point2d;
using mpc_core::relaxCurrentViolations;
using mpc_core::StateTrajectory;

namespace
{

StateTrajectory nominalAlongX(int steps, double spacing)
{
  StateTrajectory traj;
  for (int k = 0; k <= steps; ++k) {
    traj.emplace_back(spacing * k, 0.0, 0.0);
  }
  return traj;
}

// Constraints along a nominal trajectory that follows the reference.
mpc_core::StageConstraints constraintsAlong(
  const StateTrajectory & nominal, const ObstaclePoints & obstacles, const ObstacleConfig & cfg)
{
  return buildObstacleConstraints(nominal, nominal, obstacles, cfg);
}

// Costmap-like cells of a box with corners (x0, y0) and (x1, y1).
ObstaclePoints boxCells(double x0, double y0, double x1, double y1)
{
  ObstaclePoints cells;
  for (double x = x0; x <= x1 + 1e-9; x += 0.05) {
    for (double y = y0; y <= y1 + 1e-9; y += 0.05) {
      cells.emplace_back(x, y);
    }
  }
  return cells;
}

}  // namespace

TEST(ObstacleConstraints, SatisfiedByNominalWhenFarEnough)
{
  ObstacleConfig cfg;
  cfg.safety_distance = 0.3;
  const auto nominal = nominalAlongX(5, 0.1);
  const ObstaclePoints obstacles{{0.2, 0.5}};
  const auto c = constraintsAlong(nominal, obstacles, cfg);
  ASSERT_EQ(c.size(), nominal.size());
  EXPECT_TRUE(c[0].empty());
  for (std::size_t k = 1; k < c.size(); ++k) {
    ASSERT_EQ(c[k].size(), 1u);
    const auto & hp = c[k].front();
    EXPECT_NEAR(hp.normal.norm(), 1.0, 1e-12);
    // The nominal point is 0.5+ m away, so the 0.3 m constraint holds with margin.
    EXPECT_GT(hp.normal.dot(nominal[k].head<2>()), hp.offset);
  }
}

TEST(ObstacleConstraints, ViolatedWhenTooClose)
{
  ObstacleConfig cfg;
  cfg.safety_distance = 0.3;
  const auto c = constraintsAlong(nominalAlongX(1, 0.1), {{0.1, 0.1}}, cfg);
  ASSERT_EQ(c[1].size(), 1u);
  EXPECT_LT(c[1].front().normal.dot(Point2d(0.1, 0.0)), c[1].front().offset);
}

TEST(ObstacleConstraints, IgnoresPointsBeyondActivationDistance)
{
  ObstacleConfig cfg;
  cfg.activation_distance = 0.5;
  const auto c = constraintsAlong(nominalAlongX(3, 0.1), {{0.0, 2.0}}, cfg);
  for (const auto & stage : c) {
    EXPECT_TRUE(stage.empty());
  }
}

TEST(ObstacleConstraints, CorridorProducesOneConstraintPerWall)
{
  ObstacleConfig cfg;
  cfg.max_constraints_per_stage = 2;
  cfg.min_angle_separation = std::numbers::pi / 4.0;
  // Two points on the left wall (closest) and one on the right wall.
  const ObstaclePoints walls{{0.1, 0.40}, {0.15, 0.42}, {0.1, -0.5}};
  const auto c = constraintsAlong(nominalAlongX(1, 0.1), walls, cfg);
  ASSERT_EQ(c[1].size(), 2u);
  EXPECT_LT(c[1][0].normal.y(), 0.0);  // pushes away from the left wall
  EXPECT_GT(c[1][1].normal.y(), 0.0);  // pushes away from the right wall
}

TEST(ObstacleConstraints, ObstacleAheadTiltsNormalToEscapeSideways)
{
  ObstacleConfig cfg;
  cfg.escape_angle = std::numbers::pi / 4.0;
  cfg.safety_distance = 0.3;
  const ObstaclePoints ahead{{0.6, 0.0}};
  const auto c = constraintsAlong(nominalAlongX(1, 0.1), ahead, cfg);
  ASSERT_EQ(c[1].size(), 1u);
  const Point2d n = c[1].front().normal;
  EXPECT_NEAR(n.norm(), 1.0, 1e-12);
  // 45 deg between the normal and the lateral axis, tilted backwards: sliding forward-right
  // along the boundary is allowed.
  EXPECT_NEAR(std::abs(n.y()), std::cos(cfg.escape_angle), 1e-12);
  EXPECT_LT(n.x(), 0.0);
  // Whatever the tilt, every point satisfying the constraint keeps the safety distance.
  for (double x = -1.0; x <= 2.0; x += 0.05) {
    for (double y = -1.5; y <= 1.5; y += 0.05) {
      const Point2d p(x, y);
      if (n.dot(p) >= c[1].front().offset) {
        EXPECT_GE((p - ahead.front()).norm(), cfg.safety_distance - 1e-9);
      }
    }
  }
}

TEST(ObstacleConstraints, WideObstacleAheadIsPassedOnOneSide)
{
  ObstacleConfig cfg;
  cfg.safety_distance = 0.3;
  // Across the path, mostly on its left: passing on the right is the shorter detour.
  const ObstaclePoints box = boxCells(0.6, -0.07, 0.9, 0.23);
  const auto c = constraintsAlong(nominalAlongX(1, 0.1), box, cfg);
  // Escaping each cell on its own side would give two half-planes forming a wedge in front
  // of the box; all cells share the same escape normal instead, merged in one half-plane.
  ASSERT_EQ(c[1].size(), 1u);
  const auto & hp = c[1].front();
  EXPECT_LT(hp.normal.y(), 0.0);
  EXPECT_LT(hp.normal.x(), 0.0);
  for (const auto & o : box) {
    EXPECT_LE(hp.normal.dot(o) + cfg.safety_distance, hp.offset + 1e-12);
  }
}

TEST(ObstacleConstraints, ObstacleIsPassedOnTheSideCloserToThePath)
{
  ObstacleConfig cfg;
  cfg.safety_distance = 0.3;
  const auto path = nominalAlongX(1, 0.1);
  StateTrajectory nominal = path;
  for (auto & x : nominal) {
    x.y() = -0.2;  // predicted 20 cm right of the path
  }
  // Mostly right of the path: 0.37 m from the path passing on the left, 0.53 m on the right
  // (although, seen from the predicted position, the right side is closer).
  const auto c = buildObstacleConstraints(nominal, path, boxCells(0.8, -0.23, 1.1, 0.07), cfg);
  ASSERT_EQ(c[1].size(), 1u);
  EXPECT_GT(c[1].front().normal.y(), 0.0);
}

TEST(ObstacleConstraints, WallBendingInFrontIsPassedOnItsOpenSide)
{
  ObstacleConfig cfg;
  cfg.safety_distance = 0.3;
  // Wall along the right side of the robot that turns across its way. Passing on the right
  // would be the shorter detour (0.8 m from the path instead of 0.9 m), but the wall extends
  // beside the robot on that side.
  ObstaclePoints wall = boxCells(-1.0, -0.5, 0.8, -0.5);
  const ObstaclePoints front = boxCells(0.8, -0.45, 0.8, 0.6);
  wall.insert(wall.end(), front.begin(), front.end());
  const auto c = constraintsAlong(nominalAlongX(1, 0.1), wall, cfg);
  ASSERT_FALSE(c[1].empty());
  for (const auto & hp : c[1]) {
    EXPECT_GT(hp.normal.y(), 0.0);
  }
}

TEST(ObstacleConstraints, LongWallAheadKeepsOwnEscapeSides)
{
  ObstacleConfig cfg;
  cfg.safety_distance = 0.3;
  cfg.max_constraints_per_stage = 2;
  // A detour longer than the activation distance on both sides: no common side is imposed.
  const auto c = constraintsAlong(nominalAlongX(1, 0.1), boxCells(0.8, -2.0, 0.8, 2.0), cfg);
  ASSERT_EQ(c[1].size(), 2u);
  EXPECT_LT(c[1][0].normal.y() * c[1][1].normal.y(), 0.0);
}

TEST(ObstacleConstraints, TiltFollowsThePathNotThePredictedHeading)
{
  ObstacleConfig cfg;
  const auto path = nominalAlongX(1, 0.1);
  StateTrajectory turning = path;
  for (auto & x : turning) {
    x.z() = -0.5;
  }
  const ObstaclePoints ahead{{0.6, 0.0}};
  const auto expected = constraintsAlong(path, ahead, cfg);
  const auto c = buildObstacleConstraints(turning, path, ahead, cfg);
  ASSERT_EQ(c[1].size(), 1u);
  EXPECT_TRUE(c[1].front().normal.isApprox(expected[1].front().normal, 1e-12));
  EXPECT_NEAR(c[1].front().offset, expected[1].front().offset, 1e-12);
}

TEST(ObstacleConstraints, RejectsMismatchedReference)
{
  EXPECT_THROW(
    buildObstacleConstraints(nominalAlongX(2, 0.1), nominalAlongX(1, 0.1), {}, ObstacleConfig{}),
    std::invalid_argument);
}

TEST(ObstacleConstraints, RadialWhenObstacleIsBeside)
{
  ObstacleConfig cfg;
  const auto c = constraintsAlong(nominalAlongX(1, 0.1), {{0.1, 0.5}}, cfg);
  ASSERT_EQ(c[1].size(), 1u);
  EXPECT_NEAR(c[1].front().normal.x(), 0.0, 1e-12);
  EXPECT_NEAR(c[1].front().normal.y(), -1.0, 1e-12);
}

TEST(ReferenceWeightScale, RelaxesOnlySamplesInsideTheSafetyMargin)
{
  ObstacleConfig cfg;
  cfg.safety_distance = 0.3;
  cfg.blocked_reference_weight = 0.1;
  // Samples at x = 0, 0.1, ..., 1.0; obstacle 0.1 m left of the sample at x = 0.5.
  const auto scale = mpc_core::referenceWeightScale(nominalAlongX(10, 0.1), {{0.5, 0.1}}, cfg);
  ASSERT_EQ(scale.size(), 11u);
  for (std::size_t k = 0; k < scale.size(); ++k) {
    const double distance = std::hypot(0.1 * static_cast<double>(k) - 0.5, 0.1);
    EXPECT_DOUBLE_EQ(scale[k], distance < 0.3 ? 0.1 : 1.0) << "sample " << k;
  }
  EXPECT_EQ(mpc_core::referenceWeightScale(nominalAlongX(3, 0.1), {}, cfg).size(), 4u);
}

TEST(ObstacleConstraints, CurrentViolationIsRecoveredGradually)
{
  ObstacleConfig cfg;
  cfg.safety_distance = 0.3;
  // The robot is 0.2 m from the obstacle: 0.1 m inside the safety margin.
  const Point2d robot(0.0, 0.0);
  auto c = constraintsAlong(nominalAlongX(6, 0.0), {{0.0, 0.2}}, cfg);
  relaxCurrentViolations(c, robot, 4);
  const auto violation = [&](std::size_t k) {
    return c[k].front().offset - c[k].front().normal.dot(robot);
  };
  EXPECT_NEAR(violation(1), 0.025, 1e-12);  // 1/4 of the violation must be recovered
  EXPECT_NEAR(violation(2), 0.05, 1e-12);
  EXPECT_NEAR(violation(3), 0.075, 1e-12);
  EXPECT_NEAR(violation(4), 0.1, 1e-12);  // full constraint from step 4 on
  EXPECT_NEAR(violation(6), 0.1, 1e-12);
}

TEST(ObstacleConstraints, DisabledReturnsEmptyStages)
{
  ObstacleConfig cfg;
  cfg.enabled = false;
  const auto c = constraintsAlong(nominalAlongX(3, 0.1), {{0.1, 0.1}}, cfg);
  for (const auto & stage : c) {
    EXPECT_TRUE(stage.empty());
  }
}

TEST(DownsamplePoints, KeepsOnePointPerVoxel)
{
  const ObstaclePoints pts{{0.01, 0.01}, {0.02, 0.03}, {0.11, 0.01}, {-0.01, -0.01}};
  EXPECT_EQ(downsamplePoints(pts, 0.1).size(), 3u);
  EXPECT_EQ(downsamplePoints(pts, 0.0).size(), pts.size());
}

TEST(MinimumClearance, ComputesSmallestDistance)
{
  EXPECT_NEAR(minimumClearance(nominalAlongX(2, 1.0), {{1.0, 0.5}, {5.0, 5.0}}), 0.5, 1e-12);
  EXPECT_TRUE(std::isinf(minimumClearance(nominalAlongX(2, 1.0), {})));
}
