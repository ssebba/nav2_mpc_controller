#include "mpc_benchmark/metrics_recorder.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>

#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/exceptions.hpp"

#include "mpc_core/reference_builder.hpp"

namespace mpc_benchmark
{

namespace
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

double yawOf(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

MetricsRecorder::MetricsRecorder(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("metrics_recorder", options)
{
}

MetricsRecorder::CallbackReturn MetricsRecorder::on_configure(
  const rclcpp_lifecycle::State & /*state*/)
{
  param_listener_ =
    std::make_shared<metrics_recorder::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  RCLCPP_INFO(get_logger(), "Configured, run '%s'", params_.run_id.c_str());
  return CallbackReturn::SUCCESS;
}

MetricsRecorder::CallbackReturn MetricsRecorder::on_activate(
  const rclcpp_lifecycle::State & /*state*/)
{
  samples_.clear();
  compute_times_ms_.clear();
  last_cmd_ = geometry_msgs::msg::Twist();
  last_clearance_ = kNaN;
  ground_truth_.reset();

  cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    params_.cmd_topic, 10,
    [this](const geometry_msgs::msg::TwistStamped & msg) { last_cmd_ = msg.twist; });
  compute_time_sub_ = create_subscription<std_msgs::msg::Float64>(
    params_.compute_time_topic, 100,
    [this](const std_msgs::msg::Float64 & msg) { compute_times_ms_.push_back(msg.data); });
  if (!params_.scan_topic.empty()) {
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      params_.scan_topic, rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::LaserScan & msg) {
        float min_range = std::numeric_limits<float>::infinity();
        for (const float r : msg.ranges) {
          if (std::isfinite(r) && r >= msg.range_min && r <= msg.range_max) {
            min_range = std::min(min_range, r);
          }
        }
        last_clearance_ =
          std::isfinite(min_range) ? static_cast<double>(min_range) - params_.robot_radius : kNaN;
      });
  }
  if (!params_.ground_truth_topic.empty()) {
    ground_truth_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      params_.ground_truth_topic, 10,
      [this](const geometry_msgs::msg::PoseStamped & msg) { ground_truth_ = msg.pose; });
  }
  if (!params_.reference_path_topic.empty()) {
    // The reference is published once, latched.
    reference_sub_ = create_subscription<nav_msgs::msg::Path>(
      params_.reference_path_topic, rclcpp::QoS(1).transient_local(),
      [this](const nav_msgs::msg::Path & msg) {
        reference_.clear();
        for (const auto & p : msg.poses) {
          reference_.emplace_back(p.pose.position.x, p.pose.position.y, 0.0);
        }
      });
  }

  start_ = now();
  const auto period = std::chrono::duration<double>(1.0 / params_.rate);
  timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() { sample(); });
  RCLCPP_INFO(get_logger(), "Recording run '%s'", params_.run_id.c_str());
  return CallbackReturn::SUCCESS;
}

void MetricsRecorder::sample()
{
  Sample s;
  if (!params_.ground_truth_topic.empty()) {
    if (!ground_truth_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No ground truth pose yet");
      return;
    }
    s.pose = mpc_core::StateVec(
      ground_truth_->position.x, ground_truth_->position.y, yawOf(ground_truth_->orientation));
  } else {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(params_.path_frame, params_.robot_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No robot pose: %s", e.what());
      return;
    }
    s.pose = mpc_core::StateVec(
      tf.transform.translation.x, tf.transform.translation.y, yawOf(tf.transform.rotation));
  }
  s.t = (now() - start_).seconds();
  s.v_cmd = last_cmd_.linear.x;
  s.w_cmd = last_cmd_.angular.z;
  s.cross_track = reference_.empty()
                    ? kNaN
                    : mpc_core::ReferenceBuilder::closestPoint(
                        reference_, s.pose.head<2>(), std::numeric_limits<double>::infinity())
                        .distance;
  s.clearance = last_clearance_;
  samples_.push_back(s);
}

MetricsRecorder::CallbackReturn MetricsRecorder::on_deactivate(
  const rclcpp_lifecycle::State & /*state*/)
{
  timer_.reset();
  cmd_sub_.reset();
  compute_time_sub_.reset();
  scan_sub_.reset();
  reference_sub_.reset();
  ground_truth_sub_.reset();
  const bool ok = writeResults();
  RCLCPP_INFO(
    get_logger(), "Stopped run '%s': %zu samples, %zu compute times", params_.run_id.c_str(),
    samples_.size(), compute_times_ms_.size());
  return ok ? CallbackReturn::SUCCESS : CallbackReturn::FAILURE;
}

bool MetricsRecorder::writeResults() const
{
  std::error_code ec;
  std::filesystem::create_directories(params_.output_dir, ec);
  const std::filesystem::path dir(params_.output_dir);

  std::ofstream samples_file(dir / (params_.run_id + "_samples.csv"));
  std::ofstream compute_file(dir / (params_.run_id + "_compute.csv"));
  std::ofstream summary_file(dir / (params_.run_id + "_summary.csv"));
  if (!samples_file || !compute_file || !summary_file) {
    RCLCPP_ERROR(get_logger(), "Cannot write in '%s'", params_.output_dir.c_str());
    return false;
  }
  samples_file << samplesCsvHeader() << '\n';
  for (const auto & s : samples_) {
    samples_file << sampleCsvRow(s) << '\n';
  }
  compute_file << "compute_time_ms\n";
  for (const double t : compute_times_ms_) {
    compute_file << t << '\n';
  }
  const RunSummary summary = computeSummary(samples_, compute_times_ms_, 0.0);
  summary_file << "run_id," << summaryCsvHeader() << '\n'
               << params_.run_id << ',' << summaryCsvRow(summary) << '\n';
  return true;
}

MetricsRecorder::CallbackReturn MetricsRecorder::on_cleanup(
  const rclcpp_lifecycle::State & /*state*/)
{
  tf_listener_.reset();
  tf_buffer_.reset();
  param_listener_.reset();
  samples_.clear();
  compute_times_ms_.clear();
  reference_.clear();
  return CallbackReturn::SUCCESS;
}

MetricsRecorder::CallbackReturn MetricsRecorder::on_shutdown(
  const rclcpp_lifecycle::State & /*state*/)
{
  timer_.reset();
  return CallbackReturn::SUCCESS;
}

}  // namespace mpc_benchmark

RCLCPP_COMPONENTS_REGISTER_NODE(mpc_benchmark::MetricsRecorder)
