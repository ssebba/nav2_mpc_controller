#ifndef MPC_BENCHMARK__PROFILING_CONTROLLER_HPP_
#define MPC_BENCHMARK__PROFILING_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "nav2_core/controller.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "std_msgs/msg/float64.hpp"

namespace mpc_benchmark
{

/**
 * @brief Decorator that measures the compute time of any Nav2 controller.
 *
 * Loads the controller named by the `<name>.inner_plugin` parameter with pluginlib,
 * forwards every call to it and publishes the wall-clock duration of each
 * computeVelocityCommands() on `<name>/compute_time` (std_msgs/Float64, milliseconds).
 * The inner controller is configured with the same plugin name, so its parameters keep
 * their usual namespace: only `plugin` and `inner_plugin` change in the YAML.
 * This gives the same measurement for every controller of the benchmark.
 */
class ProfilingController : public nav2_core::Controller
{
public:
  ProfilingController();
  ~ProfilingController() override;

  void configure(
    const nav2::LifecycleNode::WeakPtr & parent, std::string name,
    nav2::TransformBuffer::SharedPtr tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void newPathReceived(const nav_msgs::msg::Path & raw_global_path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker, const nav_msgs::msg::Path & transformed_global_plan,
    const geometry_msgs::msg::PoseStamped & global_goal) override;

  bool cancel() override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  void reset() override;

private:
  pluginlib::ClassLoader<nav2_core::Controller> loader_;
  nav2_core::Controller::Ptr inner_;
  nav2::Publisher<std_msgs::msg::Float64>::SharedPtr compute_time_pub_;
};

}  // namespace mpc_benchmark

#endif  // MPC_BENCHMARK__PROFILING_CONTROLLER_HPP_
