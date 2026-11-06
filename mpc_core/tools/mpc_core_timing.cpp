// Offline study of solve time, tracking accuracy and obstacle handling vs. horizon length,
// without ROS. Runs every solver in closed loop on a figure eight, once on a free path and
// once with obstacle points next to the path, and prints one CSV row per
// (scenario, solver, horizon):
//
//   ros2 run mpc_core mpc_core_timing > docs/results/solver_timing.csv

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "mpc_core/closed_loop.hpp"
#include "mpc_core/mpc_solver.hpp"
#include "mpc_core/obstacle_constraints.hpp"
#include "mpc_core/path_generators.hpp"

namespace
{

double percentile(std::vector<double> v, double p)
{
  if (v.empty()) {
    return 0.0;
  }
  std::sort(v.begin(), v.end());
  const auto idx = static_cast<std::size_t>(p * static_cast<double>(v.size() - 1));
  return v[idx];
}

struct Variant
{
  std::string label;
  std::string solver;
  int sqp_iterations;
};

}  // namespace

int main()
{
  const std::vector<Variant> variants{
    {"baseline", "baseline", 1},
    {"ltv_mpc", "ltv_mpc", 1},
    {"nmpc_rti", "nmpc_sqp", 1},
    {"nmpc_sqp3", "nmpc_sqp", 3},
  };
  const std::vector<int> horizons{10, 20, 30, 40, 60};
  const auto path = mpc_core::paths::figureEight(1.5, 0.05);
  // Points ~0.25 m from the path (safety distance 0.3 m) force small, active deviations.
  const mpc_core::ObstaclePoints obstacles{{1.06, 1.0}, {-1.06, -1.0}, {0.0, 0.3}};

  struct Scenario
  {
    std::string name;
    mpc_core::ObstaclePoints obstacles;
  };
  const std::vector<Scenario> scenarios{{"free", {}}, {"obstacles", obstacles}};

  std::printf(
    "scenario,solver,horizon,mean_ms,p50_ms,p95_ms,max_ms,mean_qp_iterations,"
    "rms_cross_track_m,max_cross_track_m,min_clearance_m,progress\n");
  for (const auto & scenario : scenarios) {
    for (const auto & variant : variants) {
      for (const int horizon : horizons) {
        mpc_core::MpcConfig cfg;
        cfg.horizon = horizon;
        cfg.solver.sqp_iterations = variant.sqp_iterations;
        auto solver = mpc_core::makeSolver(variant.solver);
        solver->configure(cfg);

        // One full lap at v_ref (length ~9.2 m) plus 20% margin.
        const int cycles = static_cast<int>(1.2 * 9.2 / (cfg.v_ref * cfg.dt));
        const auto res = mpc_core::runClosedLoop(
          *solver, cfg, path, path.front(), cycles, scenario.obstacles, 0.05);

        std::vector<double> times;
        double qp_iterations = 0.0;
        for (const auto & s : res.solutions) {
          times.push_back(s.solve_time_ms);
          qp_iterations += s.qp_iterations;
        }
        double mean = 0.0;
        for (const double t : times) {
          mean += t;
        }
        const auto n = static_cast<double>(std::max<std::size_t>(1, times.size()));
        const double clearance = mpc_core::minimumClearance(res.states, scenario.obstacles);
        std::printf(
          "%s,%s,%d,%.4f,%.4f,%.4f,%.4f,%.1f,%.4f,%.4f,%.3f,%.3f\n", scenario.name.c_str(),
          variant.label.c_str(), horizon, mean / n, percentile(times, 0.5), percentile(times, 0.95),
          percentile(times, 1.0), qp_iterations / n, res.rms_cross_track, res.max_cross_track,
          std::isinf(clearance) ? -1.0 : clearance, res.progress);
      }
    }
  }
  return 0;
}
