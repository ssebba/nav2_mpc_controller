#ifndef NAV2_MPC_CONTROLLER__CONVERSIONS_HPP_
#define NAV2_MPC_CONTROLLER__CONVERSIONS_HPP_

#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "mpc_core/types.hpp"
#include "nav2_mpc_controller/mpc_controller_parameters.hpp"

namespace nav2_mpc_controller
{

// ROS message <-> mpc_core conversions. Kept free of any node state so they can be unit tested.

mpc_core::StateVec toState(const geometry_msgs::msg::Pose & pose);

mpc_core::StateTrajectory toStates(const nav_msgs::msg::Path & path);

nav_msgs::msg::Path toPathMsg(
  const mpc_core::StateTrajectory & states, const std_msgs::msg::Header & header);

geometry_msgs::msg::TwistStamped toTwist(
  const mpc_core::ControlVec & u, const std_msgs::msg::Header & header);

// Solver configuration from the ROS parameters. `max_linear_speed` is the active speed
// limit (<= params.limits.v_max), `control_period` the period of the controller server.
mpc_core::MpcConfig toMpcConfig(
  const Params & params, double max_linear_speed, double control_period);

// Boundary segments of the active half-plane constraints and the obstacle points that
// generated them, for RViz.
visualization_msgs::msg::MarkerArray toConstraintMarkers(
  const mpc_core::StageConstraints & constraints, double safety_distance,
  const std_msgs::msg::Header & header, const std::string & ns);

}  // namespace nav2_mpc_controller

#endif  // NAV2_MPC_CONTROLLER__CONVERSIONS_HPP_
