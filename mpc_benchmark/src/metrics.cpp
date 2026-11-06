#include "mpc_benchmark/metrics.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace mpc_benchmark
{

double percentile(std::vector<double> values, double p)
{
  values.erase(
    std::remove_if(values.begin(), values.end(), [](double v) { return std::isnan(v); }),
    values.end());
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const double pos = std::clamp(p, 0.0, 1.0) * static_cast<double>(values.size() - 1);
  const auto lo = static_cast<std::size_t>(std::floor(pos));
  const auto hi = std::min(lo + 1, values.size() - 1);
  const double frac = pos - static_cast<double>(lo);
  return values[lo] + frac * (values[hi] - values[lo]);
}

Stats summarize(const std::vector<double> & values)
{
  Stats s;
  double sum = 0.0;
  double sum_sq = 0.0;
  s.max = -std::numeric_limits<double>::infinity();
  for (const double v : values) {
    if (std::isnan(v)) {
      continue;
    }
    sum += v;
    sum_sq += v * v;
    s.max = std::max(s.max, v);
    ++s.count;
  }
  if (s.count == 0) {
    return {};
  }
  const auto n = static_cast<double>(s.count);
  s.mean = sum / n;
  s.rms = std::sqrt(sum_sq / n);
  s.p50 = percentile(values, 0.5);
  s.p95 = percentile(values, 0.95);
  return s;
}

double pathLength(const mpc_core::StateTrajectory & poses)
{
  double length = 0.0;
  for (std::size_t i = 1; i < poses.size(); ++i) {
    length += (poses[i].head<2>() - poses[i - 1].head<2>()).norm();
  }
  return length;
}

std::vector<double> derivative(
  const std::vector<double> & times, const std::vector<double> & values)
{
  std::vector<double> out;
  const std::size_t n = std::min(times.size(), values.size());
  if (n < 2) {
    return out;
  }
  out.reserve(n - 1);
  for (std::size_t i = 1; i < n; ++i) {
    const double dt = times[i] - times[i - 1];
    if (dt > 0.0) {
      out.push_back((values[i] - values[i - 1]) / dt);
    }
  }
  return out;
}

RunSummary computeSummary(
  const std::vector<Sample> & samples, const std::vector<double> & compute_times_ms,
  double collision_distance)
{
  RunSummary s;
  s.compute_time_ms = summarize(compute_times_ms);
  if (samples.empty()) {
    return s;
  }
  s.duration = samples.back().t - samples.front().t;

  mpc_core::StateTrajectory poses;
  std::vector<double> t;
  std::vector<double> v;
  std::vector<double> w;
  std::vector<double> cte;
  std::vector<double> clearance;
  for (const auto & sample : samples) {
    poses.push_back(sample.pose);
    t.push_back(sample.t);
    v.push_back(sample.v_cmd);
    w.push_back(sample.w_cmd);
    cte.push_back(sample.cross_track);
    clearance.push_back(sample.clearance);
    if (!std::isnan(sample.clearance) && sample.clearance < collision_distance) {
      ++s.collision_samples;
    }
  }
  s.path_length = pathLength(poses);
  s.cross_track = summarize(cte);
  s.clearance = summarize(clearance);
  s.min_clearance = std::numeric_limits<double>::quiet_NaN();
  for (const double c : clearance) {
    if (!std::isnan(c)) {
      s.min_clearance = std::isnan(s.min_clearance) ? c : std::min(s.min_clearance, c);
    }
  }

  const std::vector<double> accel = derivative(t, v);
  const std::vector<double> alpha = derivative(t, w);
  std::vector<double> t_mid(t.begin() + 1, t.end());
  const std::vector<double> jerk = derivative(t_mid, accel);
  s.rms_linear_accel = summarize(accel).rms;
  s.rms_angular_accel = summarize(alpha).rms;
  s.rms_linear_jerk = summarize(jerk).rms;
  s.mean_linear_speed = s.duration > 0.0 ? s.path_length / s.duration : 0.0;
  return s;
}

std::string summaryCsvHeader()
{
  return "duration_s,path_length_m,mean_speed_mps,cte_mean_m,cte_rms_m,cte_max_m,"
         "clearance_min_m,clearance_mean_m,collision_samples,compute_mean_ms,compute_p50_ms,"
         "compute_p95_ms,compute_max_ms,rms_linear_accel,rms_angular_accel,rms_linear_jerk";
}

std::string summaryCsvRow(const RunSummary & s)
{
  std::ostringstream os;
  os << std::fixed << std::setprecision(5) << s.duration << ',' << s.path_length << ','
     << s.mean_linear_speed << ',' << s.cross_track.mean << ',' << s.cross_track.rms << ','
     << s.cross_track.max << ',' << s.min_clearance << ',' << s.clearance.mean << ','
     << s.collision_samples << ',' << s.compute_time_ms.mean << ',' << s.compute_time_ms.p50 << ','
     << s.compute_time_ms.p95 << ',' << s.compute_time_ms.max << ',' << s.rms_linear_accel << ','
     << s.rms_angular_accel << ',' << s.rms_linear_jerk;
  return os.str();
}

std::string samplesCsvHeader() { return "t,x,y,yaw,v_cmd,w_cmd,cross_track,clearance"; }

std::string sampleCsvRow(const Sample & s)
{
  std::ostringstream os;
  os << std::fixed << std::setprecision(4) << s.t << ',' << s.pose.x() << ',' << s.pose.y() << ','
     << s.pose.z() << ',' << s.v_cmd << ',' << s.w_cmd << ',' << s.cross_track << ','
     << s.clearance;
  return os.str();
}

}  // namespace mpc_benchmark
