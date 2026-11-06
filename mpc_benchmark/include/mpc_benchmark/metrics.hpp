#ifndef MPC_BENCHMARK__METRICS_HPP_
#define MPC_BENCHMARK__METRICS_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "mpc_core/types.hpp"

namespace mpc_benchmark
{

// Pure C++ benchmark metrics (no ROS): everything the MetricsRecorder computes at the
// end of a run is implemented and unit tested here.

struct Stats
{
  double mean{0.0};
  double rms{0.0};
  double max{0.0};
  double p50{0.0};
  double p95{0.0};
  std::size_t count{0};
};

// Summary statistics; NaN values are ignored. All zeros for an empty input.
Stats summarize(const std::vector<double> & values);

// Linear-interpolated percentile, p in [0, 1].
double percentile(std::vector<double> values, double p);

// Length of a polyline.
double pathLength(const mpc_core::StateTrajectory & poses);

// Forward-difference derivative of `values` sampled at `times` (size n-1).
// Samples with a non-positive time step are skipped.
std::vector<double> derivative(
  const std::vector<double> & times, const std::vector<double> & values);

// One sample of a run, recorded at the controller rate.
struct Sample
{
  double t{0.0};            // [s] since the start of the run
  mpc_core::StateVec pose;  // robot pose in the path frame
  double v_cmd{0.0};        // commanded velocities
  double w_cmd{0.0};
  double cross_track{0.0};  // distance from the reference/global path [m] (NaN if unknown)
  double clearance{0.0};    // distance from the closest obstacle [m] (NaN if unknown)
};

struct RunSummary
{
  double duration{0.0};
  double path_length{0.0};
  Stats cross_track;
  Stats clearance;
  double min_clearance{0.0};  // NaN when no clearance was measured
  Stats compute_time_ms;
  // Smoothness of the commands: RMS of the linear/angular accelerations and of the jerk.
  double rms_linear_accel{0.0};
  double rms_angular_accel{0.0};
  double rms_linear_jerk{0.0};
  double mean_linear_speed{0.0};
  // Number of samples whose clearance is below `collision_distance`.
  std::size_t collision_samples{0};
};

RunSummary computeSummary(
  const std::vector<Sample> & samples, const std::vector<double> & compute_times_ms,
  double collision_distance);

// CSV helpers (header + row), with a fixed column order shared with tools/plot_results.py.
std::string summaryCsvHeader();
std::string summaryCsvRow(const RunSummary & s);
std::string samplesCsvHeader();
std::string sampleCsvRow(const Sample & s);

}  // namespace mpc_benchmark

#endif  // MPC_BENCHMARK__METRICS_HPP_
