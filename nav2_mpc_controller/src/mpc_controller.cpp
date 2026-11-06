#include "nav2_mpc_controller/mpc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "mpc_core/obstacle_constraints.hpp"
#include "mpc_core/reference_builder.hpp"
#include "mpc_core/rotate_to_heading.hpp"
#include "mpc_core/unicycle_model.hpp"
#include "nav2_mpc_controller/conversions.hpp"

namespace nav2_mpc_controller
{

void MpcController::configure(
  const nav2::LifecycleNode::WeakPtr & parent, std::string name,
  nav2::TransformBuffer::SharedPtr tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  if (!node) {
    throw nav2_core::ControllerException("MpcController: unable to lock the parent node");
  }
  plugin_name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // The first command is applied for one controller period: needed for its rate limits.
  double controller_frequency = 20.0;
  if (node->has_parameter("controller_frequency")) {
    node->get_parameter("controller_frequency", controller_frequency);
  }
  control_period_ = controller_frequency > 0.0 ? 1.0 / controller_frequency : 0.05;

  param_listener_ = std::make_shared<ParamListener>(node, plugin_name_);
  params_ = param_listener_->get_params();
  collision_checker_ = std::make_unique<TrajectoryCollisionChecker>(costmap_ros_);
  {
    std::scoped_lock lock(mutex_);
    applyParameters();
  }

  predicted_path_pub_ =
    node->create_publisher<nav_msgs::msg::Path>(plugin_name_ + "/predicted_path");
  reference_path_pub_ =
    node->create_publisher<nav_msgs::msg::Path>(plugin_name_ + "/reference_path");
  constraints_pub_ =
    node->create_publisher<visualization_msgs::msg::MarkerArray>(plugin_name_ + "/constraints");

  RCLCPP_INFO(
    logger_, "Configured %s: solver '%s', N = %d, dt = %.3f s, control period = %.3f s",
    plugin_name_.c_str(), solver_->name().c_str(), config_.horizon, config_.dt, control_period_);
}

void MpcController::applyParameters()
{
  config_ = toMpcConfig(params_, maxLinearSpeed(), control_period_);
  if (!solver_ || solver_->name() != params_.solver) {
    solver_ = mpc_core::makeSolver(params_.solver);
  }
  try {
    solver_->configure(config_);
  } catch (const std::invalid_argument & e) {
    throw nav2_core::ControllerException(std::string("MpcController: ") + e.what());
  }
}

double MpcController::maxLinearSpeed() const
{
  return speed_limit_ > 0.0 ? std::min(speed_limit_, params_.limits.v_max) : params_.limits.v_max;
}

void MpcController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up %s", plugin_name_.c_str());
  predicted_path_pub_.reset();
  reference_path_pub_.reset();
  constraints_pub_.reset();
  collision_checker_.reset();
  param_listener_.reset();
  std::scoped_lock lock(mutex_);
  solver_.reset();
}

void MpcController::activate()
{
  predicted_path_pub_->on_activate();
  reference_path_pub_->on_activate();
  constraints_pub_->on_activate();
}

void MpcController::deactivate()
{
  predicted_path_pub_->on_deactivate();
  reference_path_pub_->on_deactivate();
  constraints_pub_->on_deactivate();
}

void MpcController::newPathReceived(const nav_msgs::msg::Path & /*raw_global_path*/)
{
  // The warm start belongs to the previous plan.
  std::scoped_lock lock(mutex_);
  if (solver_) {
    solver_->reset();
  }
}

void MpcController::fail(const std::string & reason)
{
  // The controller server stops the robot after an exception.
  u_prev_.setZero();
  rotating_ = false;
  solver_->reset();
  throw nav2_core::NoValidControl("MpcController: " + reason);
}

geometry_msgs::msg::TwistStamped MpcController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/, const nav_msgs::msg::Path & transformed_global_plan,
  const geometry_msgs::msg::PoseStamped & /*global_goal*/)
{
  std::scoped_lock lock(mutex_);
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
    applyParameters();
    RCLCPP_INFO(logger_, "%s: parameters updated", plugin_name_.c_str());
  }
  if (transformed_global_plan.poses.empty()) {
    throw nav2_core::InvalidPath("MpcController: received an empty plan");
  }

  // Pose and plan are both expressed in the costmap global frame (e.g. odom).
  const mpc_core::StateVec x0 = toState(pose.pose);
  mpc_core::StateTrajectory path = toStates(transformed_global_plan);
  mpc_core::ReferenceBuilder::assignTangentYaw(path);
  const mpc_core::Reference reference = mpc_core::ReferenceBuilder::build(
    path, x0, config_.v_ref, config_.dt, config_.horizon, params_.path_search_distance);

  const bool publish = params_.visualize;
  if (publish && reference_path_pub_->get_subscription_count() > 0) {
    reference_path_pub_->publish(toPathMsg(reference.states, pose.header));
  }

  // In-place rotation with hysteresis when the robot faces away from the path. Near the goal
  // the MPC handles the final approach and alignment (terminal yaw cost) on its own:
  // switching between the two there makes the robot oscillate around the goal.
  const double distance_to_goal = (path.back().head<2>() - x0.head<2>()).norm();
  const bool rotation_allowed = params_.rotate_to_heading.enabled &&
                                distance_to_goal > params_.rotate_to_heading.min_distance_to_goal;
  if (!rotation_allowed) {
    rotating_ = false;
  } else {
    const double heading_error = mpc_core::normalizeAngle(reference.states.front().z() - x0.z());
    if (!rotating_ && std::abs(heading_error) > params_.rotate_to_heading.start_angle) {
      rotating_ = true;
    } else if (rotating_ && std::abs(heading_error) < params_.rotate_to_heading.stop_angle) {
      rotating_ = false;
      solver_->reset();  // the warm start is meaningless after the rotation
    }
    if (rotating_) {
      u_prev_ = mpc_core::rotateInPlaceCommand(
        heading_error, u_prev_, config_.limits, control_period_, params_.rotate_to_heading.gain);
      return toTwist(u_prev_, pose.header);
    }
  }

  mpc_core::ObstaclePoints obstacles;
  if (config_.obstacles.enabled) {
    nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*costmap->getMutex());
    obstacles = collectObstacleCells(
      *costmap, x0.head<2>(), params_.obstacles.search_radius,
      static_cast<unsigned char>(params_.obstacles.cost_threshold));
  }
  obstacles = mpc_core::downsamplePoints(obstacles, params_.obstacles.voxel_size);

  const mpc_core::MpcSolution sol = solver_->solve(x0, u_prev_, reference, obstacles);
  if (!sol.ok()) {
    fail(
      "the solver failed " + std::to_string(config_.solver.max_consecutive_failures + 1) +
      " consecutive times");
  }
  if (sol.status == mpc_core::SolveStatus::kFallback) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000, "%s: QP failed, replaying the previous solution",
      plugin_name_.c_str());
  }

  if (params_.collision_check.enabled) {
    const auto steps =
      static_cast<std::size_t>(std::ceil(params_.collision_check.max_time / config_.dt));
    if (const auto hit = collision_checker_->firstCollision(sol.predicted_states, steps + 1)) {
      fail(
        "predicted collision in " + std::to_string(static_cast<double>(*hit) * config_.dt) + " s");
    }
  }

  u_prev_ = sol.controls.front();

  if (publish) {
    if (predicted_path_pub_->get_subscription_count() > 0) {
      predicted_path_pub_->publish(toPathMsg(sol.predicted_states, pose.header));
    }
    if (constraints_pub_->get_subscription_count() > 0) {
      constraints_pub_->publish(toConstraintMarkers(
        sol.constraints, config_.obstacles.safety_distance, pose.header, plugin_name_));
    }
  }
  return toTwist(u_prev_, pose.header);
}

void MpcController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  std::scoped_lock lock(mutex_);
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    speed_limit_ = 0.0;
  } else if (percentage) {
    speed_limit_ = params_.limits.v_max * speed_limit / 100.0;
  } else {
    speed_limit_ = speed_limit;
  }
  applyParameters();
}

void MpcController::reset()
{
  std::scoped_lock lock(mutex_);
  u_prev_.setZero();
  rotating_ = false;
  if (solver_) {
    solver_->reset();
  }
}

}  // namespace nav2_mpc_controller

PLUGINLIB_EXPORT_CLASS(nav2_mpc_controller::MpcController, nav2_core::Controller)
