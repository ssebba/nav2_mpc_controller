#ifndef MPC_CORE__OBSTACLE_CONSTRAINTS_HPP_
#define MPC_CORE__OBSTACLE_CONSTRAINTS_HPP_

#include <vector>

#include "mpc_core/types.hpp"

namespace mpc_core
{

/**
 * @brief Builds linear (half-plane) obstacle constraints around a nominal trajectory.
 *
 * For each nominal position p_k (k >= 1) the closest obstacle points o within
 * `activation_distance` generate the constraint
 *
 *   n^T p_k >= n^T o + safety_distance
 *
 * For ANY unit vector n this half-plane excludes the disc of radius safety_distance
 * around o, so the constraint is always safe; n only decides which motions remain
 * allowed. The natural choice is radial, n = (p_k - o) / |p_k - o| (largest margin
 * at p_k), but for an obstacle straight ahead it points backwards and the optimum is
 * to stop in front of it: a well-known local minimum of linearized obstacle constraints.
 * Therefore the radial normal is tilted towards the lateral direction away from the
 * obstacle so that the angle between n and the lateral axis is at most `escape_angle`:
 * the half-plane boundary then becomes an escape route around the obstacle.
 *
 * The points of one obstacle must escape on the same side: otherwise the tilted half-planes
 * of points left and right of the axis intersect in a wedge that stops the robot in front of
 * a wide obstacle. Points are therefore grouped in clusters (the robot cannot pass between
 * points closer than 2 * safety_distance), and a cluster with a point ahead within
 * safety_distance of the axis of travel is passed on one side: the open side closer to the
 * path. A side is closed when the cluster also extends beside or behind the robot on that
 * side, or when passing it would take the robot farther than `activation_distance` from the
 * path (walls); with no open side every point keeps its own escape side.
 *
 * "Ahead", "lateral" and the tilt are measured along the direction of travel of the path at
 * each step (the heading of `reference`), not along the predicted heading: a tilt attached
 * to the predicted heading would rotate the escape route away each time the robot turns to
 * follow it, and the robot would stop turning in front of the obstacle.
 *
 * Coinciding normals are merged into the most restrictive half-plane; the other normals of
 * the same step are kept at least `min_angle_separation` apart so that, in a corridor, both
 * walls are represented instead of the closest one twice.
 *
 * @param nominal   positions p_k where the constraints are linearized
 * @param reference reference states (same size): direction of travel and path position
 */
StageConstraints buildObstacleConstraints(
  const StateTrajectory & nominal, const StateTrajectory & reference,
  const ObstaclePoints & obstacles, const ObstacleConfig & config);

// Cross-track weight scale of each reference sample: ObstacleConfig::blocked_reference_weight
// for the samples closer than safety_distance to an obstacle point, 1 for the others.
std::vector<double> referenceWeightScale(
  const StateTrajectory & reference, const ObstaclePoints & obstacles,
  const ObstacleConfig & config);

// Relaxes the constraints violated at `position` (see ObstacleConfig::violation_recovery_steps).
void relaxCurrentViolations(
  StageConstraints & constraints, const Point2d & position, int recovery_steps);

// Keeps one point per voxel_size x voxel_size cell (the first one found).
ObstaclePoints downsamplePoints(const ObstaclePoints & points, double voxel_size);

// Smallest distance between any point of the trajectory and any obstacle point.
double minimumClearance(const StateTrajectory & trajectory, const ObstaclePoints & obstacles);

}  // namespace mpc_core

#endif  // MPC_CORE__OBSTACLE_CONSTRAINTS_HPP_
