#include "mpc_core/obstacle_constraints.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mpc_core
{

namespace
{

// Cell of a uniform grid, both 32-bit indices packed in one 64-bit key.
std::uint64_t cellKey(std::int64_t ix, std::int64_t iy)
{
  return (static_cast<std::uint64_t>(ix) << 32U) ^ (static_cast<std::uint64_t>(iy) & 0xFFFFFFFFU);
}

std::int64_t cellIndex(double coordinate, double cell_size)
{
  return static_cast<std::int64_t>(std::floor(coordinate / cell_size));
}

// Connected components of the obstacle points, two points being connected when closer than
// `linkage` (union-find, neighbours searched on a grid of cell size `linkage`).
struct Clusters
{
  std::vector<std::size_t> label;                 // cluster of each point
  std::vector<std::vector<std::size_t>> members;  // points of each cluster
};

Clusters clusterPoints(const ObstaclePoints & points, double linkage)
{
  std::vector<std::size_t> parent(points.size());
  std::iota(parent.begin(), parent.end(), std::size_t{0});
  const auto root = [&parent](std::size_t i) {
    while (parent[i] != i) {
      parent[i] = parent[parent[i]];
      i = parent[i];
    }
    return i;
  };

  std::unordered_map<std::uint64_t, std::vector<std::size_t>> grid;
  for (std::size_t i = 0; i < points.size(); ++i) {
    grid[cellKey(cellIndex(points[i].x(), linkage), cellIndex(points[i].y(), linkage))].push_back(
      i);
  }
  const double linkage2 = linkage * linkage;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const std::int64_t ix = cellIndex(points[i].x(), linkage);
    const std::int64_t iy = cellIndex(points[i].y(), linkage);
    for (std::int64_t dx = -1; dx <= 1; ++dx) {
      for (std::int64_t dy = -1; dy <= 1; ++dy) {
        const auto cell = grid.find(cellKey(ix + dx, iy + dy));
        if (cell == grid.end()) {
          continue;
        }
        for (const std::size_t j : cell->second) {
          if (j > i && (points[i] - points[j]).squaredNorm() < linkage2) {
            parent[root(j)] = root(i);
          }
        }
      }
    }
  }

  Clusters clusters;
  clusters.label.resize(points.size());
  std::unordered_map<std::size_t, std::size_t> cluster_of_root;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto [it, inserted] = cluster_of_root.try_emplace(root(i), clusters.members.size());
    if (inserted) {
      clusters.members.emplace_back();
    }
    clusters.label[i] = it->second;
    clusters.members[it->second].push_back(i);
  }
  return clusters;
}

// Obstacle point in the frame of a nominal position, axes along the direction of travel of
// the path (x) and to its left (y).
struct LocalPoint
{
  double longitudinal;
  double lateral;
};

struct Frame
{
  Point2d origin;
  Point2d tangent;
  Point2d left;
  // Lateral offset of the origin from the path.
  double path_offset;

  LocalPoint local(const Point2d & o) const
  {
    return {(o - origin).dot(tangent), (o - origin).dot(left)};
  }
};

// Side on which the robot passes a cluster that blocks its way: +1 left, -1 right, 0 when no
// side is open and every point keeps its own escape side. A side is closed when the cluster
// extends beside or behind the robot on that side (e.g. a wall that bends in front of it) or
// when passing it would take the robot farther than the activation distance from the path
// (e.g. a long wall). Of two open sides the one closer to the path wins (ties: right): it is
// the cheaper detour for the tracking cost, and it does not depend on where the robot is, so
// every step and every control cycle make the same choice.
int passingSide(
  const std::vector<std::size_t> & members, const ObstaclePoints & obstacles, const Frame & frame,
  const ObstacleConfig & config)
{
  double min_lateral = std::numeric_limits<double>::infinity();
  double max_lateral = -std::numeric_limits<double>::infinity();
  bool left_open = true;
  bool right_open = true;
  for (const std::size_t i : members) {
    const LocalPoint q = frame.local(obstacles[i]);
    min_lateral = std::min(min_lateral, q.lateral);
    max_lateral = std::max(max_lateral, q.lateral);
    if (q.longitudinal <= 0.0) {
      (q.lateral > 0.0 ? left_open : right_open) = false;
    }
  }
  // Distance from the path of the robot passing on either side.
  const double right_detour =
    std::max(0.0, config.safety_distance - (min_lateral + frame.path_offset));
  const double left_detour =
    std::max(0.0, max_lateral + frame.path_offset + config.safety_distance);
  right_open = right_open && right_detour <= config.activation_distance;
  left_open = left_open && left_detour <= config.activation_distance;
  if (right_open && (!left_open || right_detour <= left_detour)) {
    return -1;
  }
  return left_open ? 1 : 0;
}

}  // namespace

StageConstraints buildObstacleConstraints(
  const StateTrajectory & nominal, const StateTrajectory & reference,
  const ObstaclePoints & obstacles, const ObstacleConfig & config)
{
  if (reference.size() != nominal.size()) {
    throw std::invalid_argument("buildObstacleConstraints: nominal and reference sizes differ");
  }
  StageConstraints constraints(nominal.size());
  if (!config.enabled || obstacles.empty() || config.max_constraints_per_stage <= 0) {
    return constraints;
  }

  const double activation2 = config.activation_distance * config.activation_distance;
  const double max_cos = std::cos(config.min_angle_separation);
  const double cos_escape = std::cos(config.escape_angle);
  const double sin_escape = std::sin(config.escape_angle);
  const auto max_per_stage = static_cast<std::size_t>(config.max_constraints_per_stage);
  // The robot cannot pass between two points closer than twice the safety distance.
  const Clusters clusters = clusterPoints(obstacles, 2.0 * config.safety_distance);
  constexpr int kUndecided = 2;
  std::vector<int> passing_side(clusters.members.size());

  std::vector<std::pair<double, std::size_t>> candidates;
  for (std::size_t k = 1; k < nominal.size(); ++k) {
    const Point2d tangent(std::cos(reference[k].z()), std::sin(reference[k].z()));
    const Point2d left(-tangent.y(), tangent.x());
    const Point2d origin = nominal[k].head<2>();
    const Frame frame{origin, tangent, left, (origin - reference[k].head<2>()).dot(left)};
    candidates.clear();
    for (std::size_t i = 0; i < obstacles.size(); ++i) {
      const double d2 = (frame.origin - obstacles[i]).squaredNorm();
      if (d2 < activation2) {
        candidates.emplace_back(d2, i);
      }
    }
    std::sort(candidates.begin(), candidates.end());

    // Clusters with a point that the robot would hit by driving straight on are passed on a
    // single side: escaping each point on its own side makes the half-planes of points left and
    // right of the axis intersect in a wedge that stops the robot in front of a wide obstacle.
    std::fill(passing_side.begin(), passing_side.end(), kUndecided);
    for (const auto & [d2, i] : candidates) {
      const std::size_t c = clusters.label[i];
      const LocalPoint q = frame.local(obstacles[i]);
      const bool blocking = q.longitudinal > 0.0 && std::abs(q.lateral) < config.safety_distance;
      if (blocking && passing_side[c] == kUndecided) {
        passing_side[c] = passingSide(clusters.members[c], obstacles, frame, config);
      }
    }

    auto & stage = constraints[k];
    for (const auto & [d2, i] : candidates) {
      const double d = std::sqrt(d2);
      if (d < 1e-6) {
        // The nominal position coincides with the obstacle point: no defined normal.
        continue;
      }
      const Point2d & o = obstacles[i];
      const LocalPoint q = frame.local(o);
      // Radial normal, tilted sideways for points close to the axis of travel so that its
      // angle from the lateral escape direction is at most escape_angle. Points ahead escape
      // on the passing side of their cluster when it has one, any other point away from itself.
      const bool ahead = q.longitudinal > 0.0;
      const int own_side = q.lateral >= 0.0 ? -1 : 1;
      const int forced_side = ahead ? passing_side[clusters.label[i]] : kUndecided;
      const int side = (forced_side == -1 || forced_side == 1) ? forced_side : own_side;
      Point2d normal = (frame.origin - o) / d;
      if (side != own_side || std::abs(q.lateral) < d * cos_escape) {
        normal = cos_escape * static_cast<double>(side) * frame.left +
                 sin_escape * (ahead ? -1.0 : 1.0) * tangent;
      }
      const double offset = normal.dot(o) + config.safety_distance;
      // Points escaping in the same direction share one half-plane, the most restrictive.
      const auto same = std::find_if(stage.begin(), stage.end(), [&](const HalfPlane & hp) {
        return normal.dot(hp.normal) > 1.0 - 1e-9;
      });
      if (same != stage.end()) {
        if (offset > same->offset) {
          *same = {normal, offset, o};
        }
        continue;
      }
      const bool separated = std::none_of(stage.begin(), stage.end(), [&](const HalfPlane & hp) {
        return normal.dot(hp.normal) > max_cos;
      });
      if (separated && stage.size() < max_per_stage) {
        stage.push_back({normal, offset, o});
      }
    }
  }
  return constraints;
}

std::vector<double> referenceWeightScale(
  const StateTrajectory & reference, const ObstaclePoints & obstacles,
  const ObstacleConfig & config)
{
  std::vector<double> scale(reference.size(), 1.0);
  const double safety2 = config.safety_distance * config.safety_distance;
  for (std::size_t k = 0; k < reference.size(); ++k) {
    const Point2d r = reference[k].head<2>();
    const bool blocked = std::any_of(obstacles.begin(), obstacles.end(), [&](const Point2d & o) {
      return (o - r).squaredNorm() < safety2;
    });
    if (blocked) {
      scale[k] = config.blocked_reference_weight;
    }
  }
  return scale;
}

void relaxCurrentViolations(
  StageConstraints & constraints, const Point2d & position, int recovery_steps)
{
  if (recovery_steps <= 0) {
    return;
  }
  for (std::size_t k = 1; k < constraints.size(); ++k) {
    const double remaining = 1.0 - static_cast<double>(k) / static_cast<double>(recovery_steps);
    if (remaining <= 0.0) {
      break;
    }
    for (auto & hp : constraints[k]) {
      const double violation = hp.offset - hp.normal.dot(position);
      if (violation > 0.0) {
        hp.offset -= remaining * violation;
      }
    }
  }
}

ObstaclePoints downsamplePoints(const ObstaclePoints & points, double voxel_size)
{
  if (voxel_size <= 0.0) {
    return points;
  }
  const auto key = [voxel_size](const Point2d & p) {
    return cellKey(cellIndex(p.x(), voxel_size), cellIndex(p.y(), voxel_size));
  };
  std::unordered_set<std::uint64_t> occupied;
  occupied.reserve(points.size());
  ObstaclePoints out;
  out.reserve(points.size());
  for (const auto & p : points) {
    if (occupied.insert(key(p)).second) {
      out.push_back(p);
    }
  }
  return out;
}

double minimumClearance(const StateTrajectory & trajectory, const ObstaclePoints & obstacles)
{
  double best = std::numeric_limits<double>::infinity();
  for (const auto & s : trajectory) {
    for (const auto & o : obstacles) {
      best = std::min(best, (s.head<2>() - o).norm());
    }
  }
  return best;
}

}  // namespace mpc_core
