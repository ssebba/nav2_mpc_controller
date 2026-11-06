#include "mpc_core/reference_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "mpc_core/unicycle_model.hpp"

namespace mpc_core
{

namespace
{

constexpr double kMinSegmentLength = 1e-9;

double segmentLength(const StateTrajectory & path, std::size_t seg)
{
  return (path[seg + 1].head<2>() - path[seg].head<2>()).norm();
}

// Pose at `offset` meters along segment `seg` (or the last pose past the end of the path).
StateVec interpolate(const StateTrajectory & path, std::size_t seg, double offset)
{
  if (seg + 1 >= path.size()) {
    return path.back();
  }
  const double len = segmentLength(path, seg);
  const double t = len > kMinSegmentLength ? std::clamp(offset / len, 0.0, 1.0) : 0.0;
  StateVec p;
  p.head<2>() = (1.0 - t) * path[seg].head<2>() + t * path[seg + 1].head<2>();
  // Interpolate yaw along the shortest arc.
  p.z() = path[seg].z() + t * normalizeAngle(path[seg + 1].z() - path[seg].z());
  return p;
}

// Moves (seg, offset) forward by `distance` meters along the path, saturating at the end.
void advance(const StateTrajectory & path, double distance, std::size_t & seg, double & offset)
{
  while (seg + 1 < path.size()) {
    const double len = segmentLength(path, seg);
    if (offset + distance <= len) {
      offset += distance;
      return;
    }
    distance -= len - offset;
    offset = 0.0;
    ++seg;
  }
}

}  // namespace

void ReferenceBuilder::assignTangentYaw(StateTrajectory & path)
{
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const Point2d d = path[i + 1].head<2>() - path[i].head<2>();
    if (d.norm() > kMinSegmentLength) {
      path[i].z() = std::atan2(d.y(), d.x());
    } else if (i > 0) {
      // Duplicated waypoint: inherit the previous heading.
      path[i].z() = path[i - 1].z();
    }
  }
}

ReferenceBuilder::PathPoint ReferenceBuilder::closestPoint(
  const StateTrajectory & path, const Point2d & position, double search_arc_length)
{
  if (path.empty()) {
    throw std::invalid_argument("ReferenceBuilder: empty path");
  }
  PathPoint best;
  best.distance = (path.front().head<2>() - position).norm();

  double travelled = 0.0;
  for (std::size_t seg = 0; seg + 1 < path.size(); ++seg) {
    if (seg > 0 && travelled > search_arc_length) {
      break;
    }
    const Point2d a = path[seg].head<2>();
    const Point2d ab = path[seg + 1].head<2>() - a;
    const double len2 = ab.squaredNorm();
    const double t =
      len2 > kMinSegmentLength ? std::clamp((position - a).dot(ab) / len2, 0.0, 1.0) : 0.0;
    const double dist = (a + t * ab - position).norm();
    if (dist < best.distance) {
      best = {seg, t * std::sqrt(len2), dist};
    }
    travelled += std::sqrt(len2);
  }
  return best;
}

Reference ReferenceBuilder::build(
  const StateTrajectory & path, const StateVec & robot, double v_ref, double dt, int horizon,
  double search_arc_length)
{
  if (path.empty()) {
    throw std::invalid_argument("ReferenceBuilder: empty path");
  }
  if (horizon <= 0 || dt <= 0.0) {
    throw std::invalid_argument("ReferenceBuilder: horizon and dt must be positive");
  }

  const double ds = std::abs(v_ref) * dt;
  const PathPoint start = closestPoint(path, robot.head<2>(), search_arc_length);
  std::size_t seg = start.segment;
  double offset = start.offset;

  Reference ref;
  ref.states.reserve(horizon + 1);
  for (int k = 0; k <= horizon; ++k) {
    if (k > 0) {
      advance(path, ds, seg, offset);
    }
    StateVec p = interpolate(path, seg, offset);
    // Unwrap yaw so it stays continuous with the previous sample (or the robot yaw).
    const double prev_yaw = ref.states.empty() ? robot.z() : ref.states.back().z();
    p.z() = prev_yaw + normalizeAngle(p.z() - prev_yaw);
    ref.states.push_back(p);
  }

  ref.controls.reserve(horizon);
  for (int k = 0; k < horizon; ++k) {
    const StateVec & a = ref.states[k];
    const StateVec & b = ref.states[k + 1];
    ref.controls.emplace_back((b.head<2>() - a.head<2>()).norm() / dt, (b.z() - a.z()) / dt);
  }
  return ref;
}

}  // namespace mpc_core
