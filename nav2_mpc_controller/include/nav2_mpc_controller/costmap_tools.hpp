#ifndef NAV2_MPC_CONTROLLER__COSTMAP_TOOLS_HPP_
#define NAV2_MPC_CONTROLLER__COSTMAP_TOOLS_HPP_

#include <cstddef>
#include <memory>
#include <optional>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "mpc_core/types.hpp"

namespace nav2_mpc_controller
{

// Centers of the cells with cost >= threshold within `radius` of `center` (costmap frame).
// The caller is responsible for locking the costmap mutex.
mpc_core::ObstaclePoints collectObstacleCells(
  const nav2_costmap_2d::Costmap2D & costmap, const mpc_core::Point2d & center, double radius,
  unsigned char threshold);

/**
 * @brief Checks predicted poses against the local costmap using the robot footprint.
 *
 * Circular robots (robot_radius) use the inscribed-inflation cost of the center cell,
 * polygonal ones the footprint outline, as the other Nav2 controllers do. Poses that fall
 * outside the costmap are not considered.
 */
class TrajectoryCollisionChecker
{
public:
  explicit TrajectoryCollisionChecker(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

  // Index of the first of the first `max_states` states in collision, if any.
  std::optional<std::size_t> firstCollision(
    const mpc_core::StateTrajectory & states, std::size_t max_states) const;

private:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
};

}  // namespace nav2_mpc_controller

#endif  // NAV2_MPC_CONTROLLER__COSTMAP_TOOLS_HPP_
