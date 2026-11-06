#include "mpc_core/closed_loop.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

#include "mpc_core/reference_builder.hpp"
#include "mpc_core/unicycle_model.hpp"

namespace mpc_core
{

ClosedLoopResult runClosedLoop(
  MpcSolver & solver, const MpcConfig & config, const StateTrajectory & path, const StateVec & x0,
  int max_cycles, const ObstaclePoints & obstacles, double goal_tolerance)
{
  constexpr double kSearchWindow = 1.0;  // [m], same role as the path handler pruning
  ClosedLoopResult out;
  StateVec x = x0;
  ControlVec u_prev = ControlVec::Zero();
  double sum_sq = 0.0;
  std::size_t start = 0;

  for (int i = 0; i < max_cycles; ++i) {
    const StateTrajectory remaining(path.begin() + static_cast<std::ptrdiff_t>(start), path.end());
    start += ReferenceBuilder::closestPoint(remaining, x.head<2>(), kSearchWindow).segment;
    const StateTrajectory pruned(path.begin() + static_cast<std::ptrdiff_t>(start), path.end());

    const Reference ref =
      ReferenceBuilder::build(pruned, x, config.v_ref, config.dt, config.horizon, kSearchWindow);
    MpcSolution sol = solver.solve(x, u_prev, ref, obstacles);
    if (!sol.ok()) {
      out.all_ok = false;
      break;
    }
    u_prev = sol.controls.front();
    x = UnicycleModel::step(x, u_prev, config.dt);
    out.states.push_back(x);
    out.controls.push_back(u_prev);
    out.solutions.push_back(std::move(sol));

    const double e =
      ReferenceBuilder::closestPoint(pruned, x.head<2>(), std::numeric_limits<double>::infinity())
        .distance;
    out.cross_track.push_back(e);
    out.max_cross_track = std::max(out.max_cross_track, e);
    sum_sq += e * e;

    // Closed curves end where they start: also require the last segment to be reached.
    const bool on_last_segment = start + 2 >= path.size();
    if (
      goal_tolerance > 0.0 && on_last_segment &&
      (x.head<2>() - path.back().head<2>()).norm() < goal_tolerance &&
      std::abs(u_prev.x()) < 0.01) {
      out.reached_end = true;
      break;
    }
  }
  if (!out.states.empty()) {
    out.rms_cross_track = std::sqrt(sum_sq / static_cast<double>(out.states.size()));
  }
  double total = 0.0;
  double covered = 0.0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double len = (path[i + 1].head<2>() - path[i].head<2>()).norm();
    total += len;
    if (i < start) {
      covered += len;
    }
  }
  out.progress = total > 0.0 ? covered / total : 1.0;
  if (out.reached_end) {
    out.progress = 1.0;
  }
  return out;
}

}  // namespace mpc_core
