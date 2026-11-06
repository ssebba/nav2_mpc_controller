#ifndef NAV2_MPC_CONTROLLER__MPC_CONTROLLER_HPP_
#define NAV2_MPC_CONTROLLER__MPC_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "mpc_core/mpc_solver.hpp"
#include "mpc_core/types.hpp"
#include "nav2_mpc_controller/costmap_tools.hpp"
#include "nav2_mpc_controller/mpc_controller_parameters.hpp"

namespace nav2_mpc_controller
{

/**
 * @brief Nav2 controller plugin wrapping the mpc_core solvers.
 *
 * All the math lives in the ROS-agnostic mpc_core library; this class:
 *  - declares and validates the parameters (generate_parameter_library, updatable at runtime),
 *  - converts the pruned plan into a time-parametrized reference,
 *  - extracts obstacle points from the local costmap,
 *  - rotates in place when the heading error is large (with hysteresis),
 *  - checks the predicted footprint for collisions before applying a command,
 *  - publishes debug topics for RViz.
 */
class MpcController : public nav2_core::Controller
{
public:
  MpcController() = default;
  ~MpcController() override = default;

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

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  void reset() override;

private:
  // Rebuilds the solver configuration (and the solver if its type changed). Lock held.
  void applyParameters();
  // Speed limit currently in force [m/s].
  double maxLinearSpeed() const;
  // Stops tracking state and throws: the controller server then publishes zero velocity.
  [[noreturn]] void fail(const std::string & reason);

  nav2::LifecycleNode::WeakPtr node_;
  std::string plugin_name_;
  nav2::TransformBuffer::SharedPtr tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("MpcController")};
  rclcpp::Clock::SharedPtr clock_;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
  double control_period_{0.05};

  // Guards everything below: computeVelocityCommands, setSpeedLimit and reset can be
  // called from different executor threads of the controller server.
  std::mutex mutex_;
  mpc_core::MpcConfig config_;
  std::unique_ptr<mpc_core::MpcSolver> solver_;
  std::unique_ptr<TrajectoryCollisionChecker> collision_checker_;
  mpc_core::ControlVec u_prev_{mpc_core::ControlVec::Zero()};
  bool rotating_{false};
  double speed_limit_{0.0};  // [m/s], 0 = none

  nav2::Publisher<nav_msgs::msg::Path>::SharedPtr predicted_path_pub_;
  nav2::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_pub_;
  nav2::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr constraints_pub_;
};

}  // namespace nav2_mpc_controller

#endif  // NAV2_MPC_CONTROLLER__MPC_CONTROLLER_HPP_
