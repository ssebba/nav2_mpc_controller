#include "nav2_mpc_controller/conversions.hpp"

#include <algorithm>
#include <cmath>

#include "geometry_msgs/msg/point.hpp"

namespace nav2_mpc_controller
{

namespace
{

Eigen::Vector3d toVector3(const std::vector<double> & v) { return {v.at(0), v.at(1), v.at(2)}; }

Eigen::Vector2d toVector2(const std::vector<double> & v) { return {v.at(0), v.at(1)}; }

geometry_msgs::msg::Point toPoint(const mpc_core::Point2d & p)
{
  geometry_msgs::msg::Point out;
  out.x = p.x();
  out.y = p.y();
  out.z = 0.05;
  return out;
}

}  // namespace

mpc_core::StateVec toState(const geometry_msgs::msg::Pose & pose)
{
  // Yaw of the quaternion (ZYX convention), without pulling in tf2_geometry_msgs.
  const auto & q = pose.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  return {pose.position.x, pose.position.y, yaw};
}

mpc_core::StateTrajectory toStates(const nav_msgs::msg::Path & path)
{
  mpc_core::StateTrajectory states;
  states.reserve(path.poses.size());
  for (const auto & p : path.poses) {
    states.push_back(toState(p.pose));
  }
  return states;
}

nav_msgs::msg::Path toPathMsg(
  const mpc_core::StateTrajectory & states, const std_msgs::msg::Header & header)
{
  nav_msgs::msg::Path msg;
  msg.header = header;
  msg.poses.reserve(states.size());
  for (const auto & s : states) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = header;
    ps.pose.position.x = s.x();
    ps.pose.position.y = s.y();
    ps.pose.orientation.z = std::sin(0.5 * s.z());
    ps.pose.orientation.w = std::cos(0.5 * s.z());
    msg.poses.push_back(ps);
  }
  return msg;
}

geometry_msgs::msg::TwistStamped toTwist(
  const mpc_core::ControlVec & u, const std_msgs::msg::Header & header)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header = header;
  cmd.twist.linear.x = u.x();
  cmd.twist.angular.z = u.y();
  return cmd;
}

mpc_core::MpcConfig toMpcConfig(
  const Params & params, double max_linear_speed, double control_period)
{
  mpc_core::MpcConfig cfg;
  cfg.horizon = static_cast<int>(params.horizon);
  cfg.dt = params.dt;
  cfg.control_period = control_period;

  cfg.limits.v_min = params.limits.v_min;
  cfg.limits.v_max = std::min(params.limits.v_max, max_linear_speed);
  cfg.limits.w_max = params.limits.w_max;
  cfg.limits.a_max = params.limits.a_max;
  cfg.limits.alpha_max = params.limits.alpha_max;
  cfg.v_ref = std::min(params.desired_linear_vel, cfg.limits.v_max);

  cfg.weights.q = toVector3(params.weights.q);
  cfg.weights.q_terminal = toVector3(params.weights.q_terminal);
  cfg.weights.r = toVector2(params.weights.r);
  cfg.weights.r_delta = toVector2(params.weights.r_delta);

  const auto & obs = params.obstacles;
  cfg.obstacles.enabled = obs.enabled;
  cfg.obstacles.safety_distance = obs.safety_distance;
  cfg.obstacles.activation_distance = obs.activation_distance;
  cfg.obstacles.max_constraints_per_stage = static_cast<int>(obs.max_constraints_per_stage);
  cfg.obstacles.min_angle_separation = obs.min_angle_separation;
  cfg.obstacles.escape_angle = obs.escape_angle;
  cfg.obstacles.violation_recovery_steps = static_cast<int>(obs.violation_recovery_steps);
  cfg.obstacles.blocked_reference_weight = obs.blocked_reference_weight;
  cfg.obstacles.slack_linear_weight = obs.slack_linear_weight;
  cfg.obstacles.slack_quadratic_weight = obs.slack_quadratic_weight;

  cfg.solver.sqp_iterations = static_cast<int>(params.sqp.iterations);
  cfg.solver.sqp_tolerance = params.sqp.tolerance;
  cfg.solver.linearization_min_speed = params.sqp.linearization_min_speed;
  cfg.solver.max_qp_iterations = static_cast<int>(params.qp.max_iterations);
  cfg.solver.eps_abs = params.qp.eps_abs;
  cfg.solver.eps_rel = params.qp.eps_rel;
  cfg.solver.max_solve_time = params.qp.max_solve_time;
  cfg.solver.max_consecutive_failures = static_cast<int>(params.qp.max_consecutive_failures);
  return cfg;
}

visualization_msgs::msg::MarkerArray toConstraintMarkers(
  const mpc_core::StageConstraints & constraints, double safety_distance,
  const std_msgs::msg::Header & header, const std::string & ns)
{
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker clear;
  clear.header = header;
  clear.ns = ns;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  visualization_msgs::msg::Marker planes;
  planes.header = header;
  planes.ns = ns;
  planes.id = 0;
  planes.type = visualization_msgs::msg::Marker::LINE_LIST;
  planes.action = visualization_msgs::msg::Marker::ADD;
  planes.pose.orientation.w = 1.0;
  planes.scale.x = 0.01;
  planes.color.r = 1.0F;
  planes.color.g = 0.45F;
  planes.color.a = 0.8F;

  visualization_msgs::msg::Marker points = planes;
  points.id = 1;
  points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  points.scale.x = points.scale.y = points.scale.z = 0.06;
  points.color.r = 0.9F;
  points.color.g = 0.1F;
  points.color.b = 0.1F;

  constexpr double kHalfLength = 0.15;
  for (const auto & stage : constraints) {
    for (const auto & hp : stage) {
      // The boundary touches the safety circle at o + n * d_safe.
      const mpc_core::Point2d touch = hp.obstacle + safety_distance * hp.normal;
      const mpc_core::Point2d along(-hp.normal.y(), hp.normal.x());
      planes.points.push_back(toPoint(touch - kHalfLength * along));
      planes.points.push_back(toPoint(touch + kHalfLength * along));
      points.points.push_back(toPoint(hp.obstacle));
    }
  }
  if (!planes.points.empty()) {
    markers.markers.push_back(planes);
    markers.markers.push_back(points);
  }
  return markers;
}

}  // namespace nav2_mpc_controller
