#include "mpc_benchmark/profiling_controller.hpp"

#include <chrono>
#include <string>
#include <utility>

#include "nav2_core/controller_exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mpc_benchmark
{

ProfilingController::ProfilingController() : loader_("nav2_core", "nav2_core::Controller") {}

ProfilingController::~ProfilingController()
{
  // The inner controller must be destroyed before its class loader.
  inner_.reset();
}

void ProfilingController::configure(
  const nav2::LifecycleNode::WeakPtr & parent, std::string name,
  nav2::TransformBuffer::SharedPtr tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::ControllerException("ProfilingController: unable to lock the parent node");
  }
  const std::string param = name + ".inner_plugin";
  const auto inner_type = node->declare_or_get_parameter<std::string>(param, std::string());
  if (inner_type.empty()) {
    throw nav2_core::ControllerException("ProfilingController: parameter '" + param + "' is empty");
  }
  try {
    inner_ = loader_.createSharedInstance(inner_type);
  } catch (const pluginlib::PluginlibException & e) {
    throw nav2_core::ControllerException(
      "ProfilingController: cannot load '" + inner_type + "': " + e.what());
  }
  compute_time_pub_ = node->create_publisher<std_msgs::msg::Float64>(name + "/compute_time");
  RCLCPP_INFO(node->get_logger(), "ProfilingController: profiling '%s'", inner_type.c_str());
  inner_->configure(parent, std::move(name), std::move(tf), std::move(costmap_ros));
}

void ProfilingController::cleanup()
{
  inner_->cleanup();
  compute_time_pub_.reset();
}

void ProfilingController::activate()
{
  compute_time_pub_->on_activate();
  inner_->activate();
}

void ProfilingController::deactivate()
{
  inner_->deactivate();
  compute_time_pub_->on_deactivate();
}

void ProfilingController::newPathReceived(const nav_msgs::msg::Path & raw_global_path)
{
  inner_->newPathReceived(raw_global_path);
}

geometry_msgs::msg::TwistStamped ProfilingController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker, const nav_msgs::msg::Path & transformed_global_plan,
  const geometry_msgs::msg::PoseStamped & global_goal)
{
  const auto start = std::chrono::steady_clock::now();
  // Measured also when the inner controller throws: failures cost time too.
  struct PublishOnExit
  {
    ProfilingController * self;
    std::chrono::steady_clock::time_point start;
    ~PublishOnExit()
    {
      try {
        std_msgs::msg::Float64 msg;
        msg.data =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
            .count();
        self->compute_time_pub_->publish(msg);
      } catch (const std::exception &) {  // NOLINT(bugprone-empty-catch)
        // Never throw from a destructor; a lost sample is harmless.
      }
    }
  } publish_on_exit{this, start};
  return inner_->computeVelocityCommands(
    pose, velocity, goal_checker, transformed_global_plan, global_goal);
}

bool ProfilingController::cancel() { return inner_->cancel(); }

void ProfilingController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  inner_->setSpeedLimit(speed_limit, percentage);
}

void ProfilingController::reset() { inner_->reset(); }

}  // namespace mpc_benchmark

PLUGINLIB_EXPORT_CLASS(mpc_benchmark::ProfilingController, nav2_core::Controller)
