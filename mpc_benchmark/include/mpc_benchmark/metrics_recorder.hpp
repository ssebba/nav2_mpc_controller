#ifndef MPC_BENCHMARK__METRICS_RECORDER_HPP_
#define MPC_BENCHMARK__METRICS_RECORDER_HPP_

#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "mpc_benchmark/metrics.hpp"
#include "mpc_benchmark/metrics_recorder_parameters.hpp"

namespace mpc_benchmark
{

/**
 * @brief Lifecycle component that records one benchmark run.
 *
 * The lifecycle states map onto the run: `activate` starts the recording, `deactivate`
 * computes the summary (mpc_benchmark/metrics.hpp) and writes `<run_id>_samples.csv`,
 * `<run_id>_compute.csv` (one compute time per control cycle) and `<run_id>_summary.csv`. The ScenarioRunner drives these transitions through the
 * standard lifecycle services, so the two nodes stay decoupled.
 */
class MetricsRecorder : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit MetricsRecorder(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void sample();
  bool writeResults() const;

  std::shared_ptr<metrics_recorder::ParamListener> param_listener_;
  metrics_recorder::Params params_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr compute_time_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr reference_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ground_truth_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Twist last_cmd_;
  std::optional<geometry_msgs::msg::Pose> ground_truth_;
  double last_clearance_{std::numeric_limits<double>::quiet_NaN()};
  mpc_core::StateTrajectory reference_;
  std::vector<Sample> samples_;
  std::vector<double> compute_times_ms_;
  rclcpp::Time start_;
};

}  // namespace mpc_benchmark

#endif  // MPC_BENCHMARK__METRICS_RECORDER_HPP_
