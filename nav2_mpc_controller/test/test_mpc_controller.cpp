// In-process tests of the plugin: a real nav2::LifecycleNode and Costmap2DROS, no executor,
// no TF (the plan is given in the costmap frame, as the controller server does).

#include <algorithm>
#include <cmath>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.hpp"

#include "nav2_mpc_controller/mpc_controller.hpp"

namespace
{

geometry_msgs::msg::PoseStamped makePose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(0.5 * yaw);
  p.pose.orientation.w = std::cos(0.5 * yaw);
  return p;
}

// Straight plan along +x from (x0, y) to (x1, y), 5 cm spacing.
nav_msgs::msg::Path straightPlan(double x0, double x1, double y)
{
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "map";
  for (double x = x0; x <= x1 + 1e-9; x += 0.05) {
    plan.poses.push_back(makePose(x, y, 0.0));
  }
  return plan;
}

double distance(
  const geometry_msgs::msg::PoseStamped & a, const geometry_msgs::msg::PoseStamped & b)
{
  return std::hypot(a.pose.position.x - b.pose.position.x, a.pose.position.y - b.pose.position.y);
}

class MpcControllerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides_);
    node_ = std::make_shared<nav2::LifecycleNode>("controller_server", options);
    node_->declare_parameter("controller_frequency", 20.0);

    // 5 m x 5 m costmap with origin (0, 0), 0.1 m cells and a 0.1 m robot radius (defaults).
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap", "/", false);
    costmap_->on_configure(rclcpp_lifecycle::State());
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());

    controller_ = std::make_shared<nav2_mpc_controller::MpcController>();
    controller_->configure(node_, "FollowPath", tf_, costmap_);
    controller_->activate();
  }

  void TearDown() override
  {
    controller_->deactivate();
    controller_->cleanup();
    controller_.reset();
    costmap_->shutdown();
    costmap_.reset();
    node_->shutdown();
    node_.reset();
  }

  // Integrates the commands on a unicycle for `cycles` controller periods. As the Nav2 path
  // handler does, the plan given to the controller is pruned up to the robot.
  geometry_msgs::msg::PoseStamped drive(
    geometry_msgs::msg::PoseStamped pose, const nav_msgs::msg::Path & plan, int cycles,
    std::vector<geometry_msgs::msg::TwistStamped> * commands = nullptr)
  {
    const geometry_msgs::msg::Twist unused;
    for (int i = 0; i < cycles; ++i) {
      nav_msgs::msg::Path pruned = plan;
      const auto closest = std::min_element(
        pruned.poses.begin(), pruned.poses.end(),
        [&pose](const auto & a, const auto & b) { return distance(a, pose) < distance(b, pose); });
      pruned.poses.erase(pruned.poses.begin(), closest);
      const auto cmd =
        controller_->computeVelocityCommands(pose, unused, nullptr, pruned, plan.poses.back());
      if (commands != nullptr) {
        commands->push_back(cmd);
      }
      const double yaw = 2.0 * std::atan2(pose.pose.orientation.z, pose.pose.orientation.w);
      const double dt = 0.05;
      pose = makePose(
        pose.pose.position.x + dt * cmd.twist.linear.x * std::cos(yaw),
        pose.pose.position.y + dt * cmd.twist.linear.x * std::sin(yaw),
        yaw + dt * cmd.twist.angular.z);
    }
    return pose;
  }

  void setLethal(double x, double y)
  {
    unsigned int mx = 0;
    unsigned int my = 0;
    ASSERT_TRUE(costmap_->getCostmap()->worldToMap(x, y, mx, my));
    costmap_->getCostmap()->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  std::vector<rclcpp::Parameter> overrides_;
  nav2::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_mpc_controller::MpcController> controller_;
};

TEST_F(MpcControllerTest, DeclaresParametersUnderThePluginName)
{
  EXPECT_TRUE(node_->has_parameter("FollowPath.horizon"));
  EXPECT_TRUE(node_->has_parameter("FollowPath.weights.q"));
  EXPECT_TRUE(node_->has_parameter("FollowPath.obstacles.safety_distance"));
  EXPECT_EQ(node_->get_parameter("FollowPath.solver").as_string(), "nmpc_sqp");
}

TEST_F(MpcControllerTest, DrivesForwardAlongAStraightPlan)
{
  const auto plan = straightPlan(1.0, 4.0, 2.5);
  const auto end = drive(makePose(1.0, 2.5, 0.0), plan, 60);
  EXPECT_GT(end.pose.position.x, 1.5);
  EXPECT_NEAR(end.pose.position.y, 2.5, 0.02);
}

TEST_F(MpcControllerTest, RespectsTheSpeedLimit)
{
  controller_->setSpeedLimit(0.1, false);
  std::vector<geometry_msgs::msg::TwistStamped> cmds;
  drive(makePose(1.0, 2.5, 0.0), straightPlan(1.0, 4.0, 2.5), 60, &cmds);
  for (const auto & c : cmds) {
    EXPECT_LE(c.twist.linear.x, 0.1 + 1e-6);
  }
  // Percentage of the nominal maximum speed (0.26 m/s by default).
  controller_->setSpeedLimit(50.0, true);
  cmds.clear();
  drive(makePose(1.0, 2.5, 0.0), straightPlan(1.0, 4.0, 2.5), 60, &cmds);
  EXPECT_NEAR(cmds.back().twist.linear.x, 0.13, 0.01);
}

TEST_F(MpcControllerTest, RotatesInPlaceWhenFacingAway)
{
  std::vector<geometry_msgs::msg::TwistStamped> cmds;
  const auto end =
    drive(makePose(1.0, 2.5, std::numbers::pi - 0.2), straightPlan(1.0, 4.0, 2.5), 10, &cmds);
  for (const auto & c : cmds) {
    EXPECT_NEAR(c.twist.linear.x, 0.0, 1e-9);
  }
  EXPECT_GT(std::abs(cmds.back().twist.angular.z), 0.3);
  EXPECT_NEAR(end.pose.position.x, 1.0, 1e-9);
}

TEST_F(MpcControllerTest, DoesNotRotateInPlaceNearTheGoal)
{
  // Goal 0.3 m ahead, robot facing away: the MPC turns while moving instead of rotating in
  // place, which would fight with the final approach.
  std::vector<geometry_msgs::msg::TwistStamped> cmds;
  drive(makePose(1.0, 2.5, std::numbers::pi - 0.2), straightPlan(1.0, 1.3, 2.5), 5, &cmds);
  double max_abs_v = 0.0;
  for (const auto & c : cmds) {
    max_abs_v = std::max(max_abs_v, std::abs(c.twist.linear.x));
  }
  EXPECT_GT(max_abs_v, 1e-3);
}

TEST_F(MpcControllerTest, EmptyPlanThrows)
{
  const nav_msgs::msg::Path empty;
  const geometry_msgs::msg::Twist twist;
  EXPECT_THROW(
    controller_->computeVelocityCommands(
      makePose(1.0, 1.0, 0.0), twist, nullptr, empty, geometry_msgs::msg::PoseStamped()),
    nav2_core::InvalidPath);
}

TEST_F(MpcControllerTest, RejectsInvalidParametersAndAppliesValidOnes)
{
  EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("FollowPath.horizon", 1)).successful);
  EXPECT_FALSE(
    node_->set_parameter(rclcpp::Parameter("FollowPath.solver", std::string("magic"))).successful);
  EXPECT_FALSE(
    node_->set_parameter(rclcpp::Parameter("FollowPath.weights.q", std::vector<double>{1.0}))
      .successful);
  EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("FollowPath.solver", std::string("ltv_mpc")))
                .successful);
  EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("FollowPath.horizon", 30)).successful);
  // The new configuration is picked up at the next cycle.
  const auto end = drive(makePose(1.0, 2.5, 0.0), straightPlan(1.0, 4.0, 2.5), 40);
  EXPECT_GT(end.pose.position.x, 1.3);
}

TEST_F(MpcControllerTest, KeepsClearanceFromCostmapObstacles)
{
  // Obstacle cell 0.2 m to the left of the plan; safety distance 0.3 m.
  setLethal(2.0, 2.7);
  std::vector<double> clearance;
  auto pose = makePose(1.0, 2.5, 0.0);
  const auto plan = straightPlan(1.0, 4.0, 2.5);
  // 8 s at 20 Hz: ~2 m at 0.25 m/s, well past the obstacle.
  for (int i = 0; i < 160; ++i) {
    pose = drive(pose, plan, 1);
    clearance.push_back(std::hypot(pose.pose.position.x - 2.05, pose.pose.position.y - 2.75));
  }
  // Cell centers are on a 0.1 m grid: the obstacle point is (2.05, 2.75).
  EXPECT_GT(*std::min_element(clearance.begin(), clearance.end()), 0.3 - 0.03);
  EXPECT_GT(pose.pose.position.x, 2.6);
  EXPECT_NEAR(pose.pose.position.y, 2.5, 0.03) << "back on the plan after the obstacle";
}

TEST_F(MpcControllerTest, PredictedCollisionThrowsWhenAvoidanceIsDisabled)
{
  ASSERT_TRUE(
    node_->set_parameter(rclcpp::Parameter("FollowPath.obstacles.enabled", false)).successful);
  // Wall across the plan 0.3 m ahead.
  for (double y = 2.0; y <= 3.0; y += 0.1) {
    setLethal(1.3, y);
  }
  const auto plan = straightPlan(1.0, 4.0, 2.5);
  EXPECT_THROW(drive(makePose(1.0, 2.5, 0.0), plan, 40), nav2_core::NoValidControl);
}

}  // namespace
