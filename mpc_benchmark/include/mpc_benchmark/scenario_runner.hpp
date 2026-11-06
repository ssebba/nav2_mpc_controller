#ifndef MPC_BENCHMARK__SCENARIO_RUNNER_HPP_
#define MPC_BENCHMARK__SCENARIO_RUNNER_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "mpc_benchmark/scenario_runner_parameters.hpp"
#include "mpc_core/types.hpp"

namespace mpc_benchmark
{

// Reference path of a follow_path scenario, expressed in the robot start frame (the path
// starts at the origin heading along +x, i.e. at the robot when the path frame is odom).
mpc_core::StateTrajectory makeReferencePath(const scenario_runner::Params & params);

/**
 * @brief Lifecycle component that executes one benchmark scenario and then shuts down.
 *
 * Once active it waits until the Nav2 server it talks to (controller_server or bt_navigator)
 * is in the ACTIVE lifecycle state, configures and activates the MetricsRecorder through its
 * lifecycle services, sends either a FollowPath goal (controller only, generated
 * reference path) or a NavigateThroughPoses goal (full stack), enforces the timeout,
 * deactivates the recorder (which writes the metrics) and appends one row to results.csv.
 * Everything is asynchronous: no callback ever blocks the executor.
 */
class ScenarioRunner : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using FollowPath = nav2_msgs::action::FollowPath;
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;

  explicit ScenarioRunner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

private:
  enum class Phase
  {
    kWaitForServers,
    kSettling,
    kStartingRecorder,
    kRunning,
    kStoppingRecorder,
    kDone,
  };

  void tick();
  void changeRecorderState(std::uint8_t transition, std::function<void(bool)> done);
  void sendGoal();
  void finish(bool success, int error_code, const std::string & message);
  void writeResult() const;

  std::shared_ptr<scenario_runner::ParamListener> param_listener_;
  scenario_runner::Params params_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr through_poses_client_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr recorder_client_;
  // Nav2 server queried for its lifecycle state: an action server is advertised (and
  // rejects goals) before the lifecycle manager activates it.
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr nav2_state_client_;
  bool nav2_active_{false};
  bool state_request_pending_{false};
  // A request sent while the service is still being matched can be lost: resend it.
  // Steady clock: the simulation clock may not be running yet.
  std::chrono::steady_clock::time_point state_request_time_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr reference_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  Phase phase_{Phase::kWaitForServers};
  rclcpp::Time phase_start_;
  rclcpp::Time goal_start_;
  bool goal_finished_{false};
  bool success_{false};
  int error_code_{0};
  std::string message_;
  double duration_{0.0};
  std::function<void()> cancel_goal_;
};

}  // namespace mpc_benchmark

#endif  // MPC_BENCHMARK__SCENARIO_RUNNER_HPP_
