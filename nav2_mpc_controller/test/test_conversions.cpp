#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

#include "nav2_mpc_controller/conversions.hpp"

using nav2_mpc_controller::Params;

TEST(Conversions, PoseToStateExtractsYaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = -2.0;
  pose.orientation.z = std::sin(0.4);
  pose.orientation.w = std::cos(0.4);
  const auto s = nav2_mpc_controller::toState(pose);
  EXPECT_DOUBLE_EQ(s.x(), 1.0);
  EXPECT_DOUBLE_EQ(s.y(), -2.0);
  EXPECT_NEAR(s.z(), 0.8, 1e-12);
}

TEST(Conversions, PathRoundTrip)
{
  const mpc_core::StateTrajectory states{{0.0, 0.0, 0.0}, {1.0, 0.5, 2.0}, {2.0, 1.0, -2.5}};
  std_msgs::msg::Header header;
  header.frame_id = "odom";
  const auto msg = nav2_mpc_controller::toPathMsg(states, header);
  ASSERT_EQ(msg.poses.size(), 3u);
  EXPECT_EQ(msg.poses[1].header.frame_id, "odom");
  const auto back = nav2_mpc_controller::toStates(msg);
  for (std::size_t i = 0; i < states.size(); ++i) {
    EXPECT_TRUE(back[i].isApprox(states[i], 1e-12)) << i;
  }
}

TEST(Conversions, ConfigFromDefaultParameters)
{
  const Params params;
  const auto cfg = nav2_mpc_controller::toMpcConfig(params, params.limits.v_max, 0.05);
  EXPECT_EQ(cfg.horizon, params.horizon);
  EXPECT_DOUBLE_EQ(cfg.dt, params.dt);
  EXPECT_DOUBLE_EQ(cfg.control_period, 0.05);
  EXPECT_DOUBLE_EQ(cfg.firstStepPeriod(), 0.05);
  EXPECT_DOUBLE_EQ(cfg.weights.q(0), params.weights.q[0]);
  EXPECT_DOUBLE_EQ(cfg.weights.r_delta(1), params.weights.r_delta[1]);
  EXPECT_EQ(cfg.obstacles.max_constraints_per_stage, params.obstacles.max_constraints_per_stage);
  EXPECT_EQ(cfg.solver.sqp_iterations, params.sqp.iterations);
}

TEST(Conversions, SpeedLimitCapsVmaxAndReference)
{
  Params params;
  params.limits.v_max = 0.5;
  params.desired_linear_vel = 0.4;
  const auto cfg = nav2_mpc_controller::toMpcConfig(params, 0.2, 0.05);
  EXPECT_DOUBLE_EQ(cfg.limits.v_max, 0.2);
  EXPECT_DOUBLE_EQ(cfg.v_ref, 0.2);
}

TEST(Conversions, ConstraintMarkers)
{
  mpc_core::StageConstraints constraints(3);
  constraints[1].push_back({{0.0, -1.0}, 0.0, {1.0, 0.5}});
  constraints[2].push_back({{0.0, -1.0}, 0.0, {1.0, 0.5}});
  std_msgs::msg::Header header;
  const auto markers = nav2_mpc_controller::toConstraintMarkers(constraints, 0.3, header, "mpc");
  ASSERT_EQ(markers.markers.size(), 3u);  // clear + planes + points
  EXPECT_EQ(markers.markers[0].action, visualization_msgs::msg::Marker::DELETEALL);
  EXPECT_EQ(markers.markers[1].points.size(), 4u);
  EXPECT_EQ(markers.markers[2].points.size(), 2u);
  // The boundary touches the safety circle: 0.3 m below the obstacle.
  EXPECT_NEAR(markers.markers[1].points[0].y, 0.2, 1e-12);

  const auto empty =
    nav2_mpc_controller::toConstraintMarkers(mpc_core::StageConstraints(3), 0.3, header, "mpc");
  EXPECT_EQ(empty.markers.size(), 1u);
}
