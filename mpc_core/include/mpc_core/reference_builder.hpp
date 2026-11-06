#ifndef MPC_CORE__REFERENCE_BUILDER_HPP_
#define MPC_CORE__REFERENCE_BUILDER_HPP_

#include <cstddef>

#include "mpc_core/types.hpp"

namespace mpc_core
{

/**
 * @brief Converts a geometric path into a time-parametrized reference for the MPC.
 *
 * Starting from the projection of the robot on the path, it samples N+1 poses
 * spaced by v_ref * dt along the arc length (linear interpolation between
 * waypoints). Once the end of the path is reached the last pose is repeated,
 * so the MPC naturally decelerates towards the goal. The feed-forward commands
 * are the velocities implied by consecutive samples.
 *
 * Reference yaw angles are unwrapped to be continuous and close to the current
 * robot yaw, so the yaw error never jumps by 2*pi.
 */
class ReferenceBuilder
{
public:
  // Location on a polyline: segment index, distance travelled along that segment
  // and euclidean distance from the query point.
  struct PathPoint
  {
    std::size_t segment{0};
    double offset{0.0};
    double distance{0.0};
  };

  // Replaces the yaw of every pose but the last one with the direction of the path tangent.
  // Planners do not always fill orientations; the last pose keeps the goal orientation.
  static void assignTangentYaw(StateTrajectory & path);

  // Closest point on the polyline. Only the first `search_arc_length` meters are searched,
  // so self-intersecting paths (e.g. a figure eight) do not make the reference jump ahead.
  static PathPoint closestPoint(
    const StateTrajectory & path, const Point2d & position, double search_arc_length);

  static Reference build(
    const StateTrajectory & path, const StateVec & robot, double v_ref, double dt, int horizon,
    double search_arc_length = 1.0);
};

}  // namespace mpc_core

#endif  // MPC_CORE__REFERENCE_BUILDER_HPP_
