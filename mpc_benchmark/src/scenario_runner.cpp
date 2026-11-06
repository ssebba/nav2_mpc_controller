#include "mpc_benchmark/scenario_runner.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <utility>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "mpc_core/path_generators.hpp"

namespace mpc_benchmark
{

using namespace std::chrono_literals;

namespace
{

geometry_msgs::msg::PoseStamped toPose(double x, double y, double yaw, const std::string & frame)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = frame;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(0.5 * yaw);
  p.pose.orientation.w = std::cos(0.5 * yaw);
  return p;
}

}  // namespace

mpc_core::StateTrajectory makeReferencePath(const scenario_runner::Params & params)
{
  const auto & p = params.path;
  mpc_core::StateTrajectory path;
  if (p.shape == "figure_eight") {
    path = mpc_core::paths::figureEight(p.size, p.step);
  } else if (p.shape == "sine") {
    path = mpc_core::paths::sineWave(p.length, p.size, p.wavelength, p.step);
  } else {
    path = mpc_core::paths::straight(p.length, p.step);
  }
  // Rotate so that the path starts heading along +x: the robot starts at rest on it.
  const double yaw0 = path.front().z();
  const double c = std::cos(-yaw0);
  const double s = std::sin(-yaw0);
  for (auto & pose : path) {
    const double x = pose.x();
    const double y = pose.y();
    pose.x() = c * x - s * y;
    pose.y() = s * x + c * y;
    pose.z() -= yaw0;
  }
  return path;
}

ScenarioRunner::ScenarioRunner(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("scenario_runner", options)
{
}

ScenarioRunner::CallbackReturn ScenarioRunner::on_configure(
  const rclcpp_lifecycle::State & /*state*/)
{
  param_listener_ =
    std::make_shared<scenario_runner::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();
  if (params_.type == "navigate_through_poses" && params_.waypoints.size() % 3 != 0) {
    RCLCPP_ERROR(get_logger(), "'waypoints' must contain [x, y, yaw] triplets");
    return CallbackReturn::FAILURE;
  }
  follow_path_client_ = rclcpp_action::create_client<FollowPath>(this, "follow_path");
  through_poses_client_ =
    rclcpp_action::create_client<NavigateThroughPoses>(this, "navigate_through_poses");
  recorder_client_ =
    create_client<lifecycle_msgs::srv::ChangeState>("/" + params_.recorder_node + "/change_state");
  const std::string nav2_node =
    params_.type == "follow_path" ? "controller_server" : "bt_navigator";
  nav2_state_client_ = create_client<lifecycle_msgs::srv::GetState>("/" + nav2_node + "/get_state");
  reference_pub_ = create_publisher<nav_msgs::msg::Path>(
    "benchmark/reference_path", rclcpp::QoS(1).transient_local());
  return CallbackReturn::SUCCESS;
}

ScenarioRunner::CallbackReturn ScenarioRunner::on_activate(
  const rclcpp_lifecycle::State & /*state*/)
{
  reference_pub_->on_activate();
  phase_ = Phase::kWaitForServers;
  phase_start_ = now();
  // Wall timer: the sim clock may not be running yet while Gazebo starts.
  timer_ = create_wall_timer(100ms, [this]() { tick(); });
  RCLCPP_INFO(
    get_logger(), "Scenario '%s' with controller '%s' (run '%s')", params_.scenario.c_str(),
    params_.controller.c_str(), params_.run_id.c_str());
  return CallbackReturn::SUCCESS;
}

void ScenarioRunner::tick()
{
  switch (phase_) {
    case Phase::kWaitForServers: {
      if (
        state_request_pending_ &&
        std::chrono::steady_clock::now() - state_request_time_ > std::chrono::seconds(2)) {
        nav2_state_client_->prune_pending_requests();
        state_request_pending_ = false;
      }
      if (!nav2_active_ && !state_request_pending_ && nav2_state_client_->service_is_ready()) {
        state_request_pending_ = true;
        state_request_time_ = std::chrono::steady_clock::now();
        nav2_state_client_->async_send_request(
          std::make_shared<lifecycle_msgs::srv::GetState::Request>(),
          // NOLINTNEXTLINE(performance-unnecessary-value-param): signature required by rclcpp.
          [this](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture f) {
            nav2_active_ =
              f.get()->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
            state_request_pending_ = false;
          });
      }
      const bool action_ready = params_.type == "follow_path"
                                  ? follow_path_client_->action_server_is_ready()
                                  : through_poses_client_->action_server_is_ready();
      if (nav2_active_ && action_ready && recorder_client_->service_is_ready()) {
        RCLCPP_INFO(get_logger(), "Nav2 is active, starting in %.1f s", params_.start_delay);
        phase_ = Phase::kSettling;
        phase_start_ = now();
      }
      break;
    }
    case Phase::kSettling:
      if ((now() - phase_start_).seconds() >= params_.start_delay) {
        phase_ = Phase::kStartingRecorder;
        changeRecorderState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, [this](bool ok) {
          if (!ok) {
            finish(false, -1, "metrics recorder failed to configure");
            return;
          }
          changeRecorderState(
            lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, [this](bool ok2) {
              if (ok2) {
                sendGoal();
              } else {
                finish(false, -1, "metrics recorder failed to activate");
              }
            });
        });
      }
      break;
    case Phase::kRunning:
      if (!goal_finished_ && (now() - goal_start_).seconds() > params_.timeout) {
        RCLCPP_WARN(get_logger(), "Timeout after %.0f s, canceling the goal", params_.timeout);
        if (cancel_goal_) {
          cancel_goal_();
        }
        finish(false, -2, "timeout");
      }
      break;
    case Phase::kStartingRecorder:
    case Phase::kStoppingRecorder:
    case Phase::kDone:
      break;
  }
}

void ScenarioRunner::changeRecorderState(std::uint8_t transition, std::function<void(bool)> done)
{
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;
  recorder_client_->async_send_request(
    request,
    // rclcpp requires this exact signature (future by value).
    // NOLINTNEXTLINE(performance-unnecessary-value-param)
    [done = std::move(done)](rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture f) {
      done(f.get()->success);
    });
}

void ScenarioRunner::sendGoal()
{
  phase_ = Phase::kRunning;
  goal_start_ = now();
  goal_finished_ = false;

  if (params_.type == "follow_path") {
    const mpc_core::StateTrajectory path = makeReferencePath(params_);
    nav_msgs::msg::Path msg;
    msg.header.frame_id = params_.frame;
    msg.header.stamp = now();
    for (const auto & s : path) {
      msg.poses.push_back(toPose(s.x(), s.y(), s.z(), params_.frame));
    }
    reference_pub_->publish(msg);

    FollowPath::Goal goal;
    goal.path = msg;
    goal.controller_id = params_.controller_id;
    rclcpp_action::Client<FollowPath>::SendGoalOptions options;
    options.goal_response_callback = [this](const auto & handle) {
      if (!handle) {
        finish(false, -3, "FollowPath goal rejected");
        return;
      }
      cancel_goal_ = [this, handle]() { follow_path_client_->async_cancel_goal(handle); };
    };
    options.result_callback = [this](const auto & result) {
      const bool ok = result.code == rclcpp_action::ResultCode::SUCCEEDED;
      finish(ok, ok ? 0 : static_cast<int>(result.result->error_code), result.result->error_msg);
    };
    follow_path_client_->async_send_goal(goal, options);
    RCLCPP_INFO(get_logger(), "Following a %zu-pose reference path", path.size());
    return;
  }

  NavigateThroughPoses::Goal goal;
  goal.poses.header.frame_id = params_.frame;
  goal.poses.header.stamp = now();
  for (std::size_t i = 0; i + 2 < params_.waypoints.size(); i += 3) {
    auto pose = toPose(
      params_.waypoints[i], params_.waypoints[i + 1], params_.waypoints[i + 2], params_.frame);
    pose.header.stamp = goal.poses.header.stamp;
    goal.poses.goals.push_back(pose);
  }
  rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions options;
  options.goal_response_callback = [this](const auto & handle) {
    if (!handle) {
      finish(false, -3, "NavigateThroughPoses goal rejected");
      return;
    }
    cancel_goal_ = [this, handle]() { through_poses_client_->async_cancel_goal(handle); };
  };
  options.result_callback = [this](const auto & result) {
    const bool ok = result.code == rclcpp_action::ResultCode::SUCCEEDED;
    finish(ok, ok ? 0 : static_cast<int>(result.result->error_code), result.result->error_msg);
  };
  through_poses_client_->async_send_goal(goal, options);
  RCLCPP_INFO(get_logger(), "Navigating through %zu waypoints", goal.poses.goals.size());
}

void ScenarioRunner::finish(bool success, int error_code, const std::string & message)
{
  if (goal_finished_ || phase_ == Phase::kStoppingRecorder || phase_ == Phase::kDone) {
    return;
  }
  goal_finished_ = true;
  success_ = success;
  error_code_ = error_code;
  message_ = message;
  duration_ = phase_ == Phase::kRunning ? (now() - goal_start_).seconds() : 0.0;
  RCLCPP_INFO(
    get_logger(), "Run finished: %s (code %d) in %.1f s %s", success ? "SUCCESS" : "FAILURE",
    error_code, duration_, message.c_str());

  phase_ = Phase::kStoppingRecorder;
  changeRecorderState(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, [this](bool ok) {
    if (!ok) {
      RCLCPP_WARN(get_logger(), "The metrics recorder failed to write its results");
    }
    writeResult();
    phase_ = Phase::kDone;
    // One scenario per process: the benchmark launch file shuts down when the runner exits.
    rclcpp::shutdown();
  });
}

void ScenarioRunner::writeResult() const
{
  std::error_code ec;
  std::filesystem::create_directories(params_.output_dir, ec);
  const auto file = std::filesystem::path(params_.output_dir) / "results.csv";
  const bool new_file = !std::filesystem::exists(file);
  std::ofstream out(file, std::ios::app);
  if (new_file) {
    out << "run_id,scenario,controller,success,error_code,duration_s,message\n";
  }
  std::string clean_message = message_;
  for (auto & ch : clean_message) {
    if (ch == ',' || ch == '\n') {
      ch = ' ';
    }
  }
  out << params_.run_id << ',' << params_.scenario << ',' << params_.controller << ','
      << (success_ ? 1 : 0) << ',' << error_code_ << ',' << duration_ << ',' << clean_message
      << '\n';
}

ScenarioRunner::CallbackReturn ScenarioRunner::on_deactivate(
  const rclcpp_lifecycle::State & /*state*/)
{
  timer_.reset();
  reference_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

ScenarioRunner::CallbackReturn ScenarioRunner::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  follow_path_client_.reset();
  through_poses_client_.reset();
  recorder_client_.reset();
  nav2_state_client_.reset();
  reference_pub_.reset();
  param_listener_.reset();
  return CallbackReturn::SUCCESS;
}

}  // namespace mpc_benchmark

RCLCPP_COMPONENTS_REGISTER_NODE(mpc_benchmark::ScenarioRunner)
