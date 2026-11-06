#include "nav2_mpc_controller/costmap_tools.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"

namespace nav2_mpc_controller
{

mpc_core::ObstaclePoints collectObstacleCells(
  const nav2_costmap_2d::Costmap2D & costmap, const mpc_core::Point2d & center, double radius,
  unsigned char threshold)
{
  int x0 = 0;
  int y0 = 0;
  int x1 = 0;
  int y1 = 0;
  costmap.worldToMapEnforceBounds(center.x() - radius, center.y() - radius, x0, y0);
  costmap.worldToMapEnforceBounds(center.x() + radius, center.y() + radius, x1, y1);

  const double radius2 = radius * radius;
  mpc_core::ObstaclePoints points;
  for (int my = y0; my <= y1; ++my) {
    for (int mx = x0; mx <= x1; ++mx) {
      const auto umx = static_cast<unsigned int>(mx);
      const auto umy = static_cast<unsigned int>(my);
      const unsigned char cost = costmap.getCost(umx, umy);
      if (cost < threshold || cost == nav2_costmap_2d::NO_INFORMATION) {
        continue;
      }
      double wx = 0.0;
      double wy = 0.0;
      costmap.mapToWorld(umx, umy, wx, wy);
      const mpc_core::Point2d p(wx, wy);
      if ((p - center).squaredNorm() <= radius2) {
        points.push_back(p);
      }
    }
  }
  return points;
}

TrajectoryCollisionChecker::TrajectoryCollisionChecker(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
: costmap_ros_(std::move(costmap_ros))
{
}

std::optional<std::size_t> TrajectoryCollisionChecker::firstCollision(
  const mpc_core::StateTrajectory & states, std::size_t max_states) const
{
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());

  const bool use_radius = costmap_ros_->getUseRadius();
  const bool track_unknown = costmap_ros_->getLayeredCostmap()->isTrackingUnknown();
  const auto footprint = costmap_ros_->getRobotFootprint();
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(costmap);

  const std::size_t n = std::min(states.size(), max_states);
  for (std::size_t i = 0; i < n; ++i) {
    const auto & s = states[i];
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap->worldToMap(s.x(), s.y(), mx, my)) {
      continue;
    }
    if (use_radius) {
      const unsigned char cost = costmap->getCost(mx, my);
      if (cost == nav2_costmap_2d::NO_INFORMATION) {
        if (track_unknown) {
          return i;
        }
        continue;
      }
      if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        return i;
      }
    } else {
      const double cost = checker.footprintCostAtPose(s.x(), s.y(), s.z(), footprint);
      if (cost == static_cast<double>(nav2_costmap_2d::NO_INFORMATION)) {
        if (track_unknown) {
          return i;
        }
        continue;
      }
      if (cost >= static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE)) {
        return i;
      }
    }
  }
  return std::nullopt;
}

}  // namespace nav2_mpc_controller
