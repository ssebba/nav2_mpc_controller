// The profiling decorator around the real MPC plugin, loaded through pluginlib.

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_ros/buffer.hpp"

#include "mpc_benchmark/profiling_controller.hpp"

namespace
{

geometry_msgs::msg::PoseStamped makePose(double x, double y)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.w = 1.0;
  return p;
}

class ProfilingControllerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap", "/", false);
    costmap_->on_configure(rclcpp_lifecycle::State());
  }

  void TearDown() override
  {
    costmap_->shutdown();
    costmap_.reset();
  }

  nav2::LifecycleNode::SharedPtr makeNode(const std::string & inner_plugin)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("FollowPath.inner_plugin", inner_plugin)});
    auto node = std::make_shared<nav2::LifecycleNode>("controller_server", options);
    node->declare_parameter("controller_frequency", 20.0);
    return node;
  }

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
};

TEST_F(ProfilingControllerTest, ForwardsToTheInnerControllerAndPublishesComputeTime)
{
  auto node = makeNode("nav2_mpc_controller::MpcController");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  mpc_benchmark::ProfilingController controller;
  controller.configure(node, "FollowPath", tf, costmap_);
  // The inner controller declared its own parameters under the same plugin name.
  EXPECT_TRUE(node->has_parameter("FollowPath.horizon"));
  controller.activate();

  std::vector<double> times;
  // nav2::LifecycleNode: topic, callback, QoS (Nav2 defaults).
  auto sub = node->create_subscription<std_msgs::msg::Float64>(
    "FollowPath/compute_time",
    [&times](const std_msgs::msg::Float64 & msg) { times.push_back(msg.data); });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  nav_msgs::msg::Path plan;
  plan.header.frame_id = "map";
  for (double x = 1.0; x <= 3.0; x += 0.05) {
    plan.poses.push_back(makePose(x, 2.5));
  }
  const geometry_msgs::msg::Twist velocity;
  double v = 0.0;
  for (int i = 0; i < 10; ++i) {
    v = controller
          .computeVelocityCommands(makePose(1.0, 2.5), velocity, nullptr, plan, plan.poses.back())
          .twist.linear.x;
    executor.spin_some();
  }
  EXPECT_GT(v, 0.0);
  ASSERT_FALSE(times.empty());
  for (const double t : times) {
    EXPECT_GT(t, 0.0);
    EXPECT_LT(t, 1000.0);
  }
  controller.deactivate();
  controller.cleanup();
  node->shutdown();
}

TEST_F(ProfilingControllerTest, UnknownInnerPluginThrows)
{
  auto node = makeNode("does_not_exist::Controller");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  mpc_benchmark::ProfilingController controller;
  EXPECT_THROW(
    controller.configure(node, "FollowPath", tf, costmap_), nav2_core::ControllerException);
  node->shutdown();
}

}  // namespace
