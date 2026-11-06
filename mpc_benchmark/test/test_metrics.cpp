#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "mpc_benchmark/metrics.hpp"

using namespace mpc_benchmark;  // NOLINT(google-build-using-namespace)

namespace
{

std::size_t columns(const std::string & csv_line)
{
  return static_cast<std::size_t>(std::count(csv_line.begin(), csv_line.end(), ',')) + 1;
}

}  // namespace

TEST(Metrics, SummarizeIgnoresNaN)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const Stats s = summarize({1.0, nan, 3.0});
  EXPECT_EQ(s.count, 2u);
  EXPECT_DOUBLE_EQ(s.mean, 2.0);
  EXPECT_DOUBLE_EQ(s.max, 3.0);
  EXPECT_DOUBLE_EQ(s.rms, std::sqrt(5.0));
  EXPECT_DOUBLE_EQ(s.p50, 2.0);
  EXPECT_EQ(summarize({}).count, 0u);
  EXPECT_DOUBLE_EQ(summarize({nan}).mean, 0.0);
}

TEST(Metrics, PercentileInterpolates)
{
  const std::vector<double> v{4.0, 1.0, 3.0, 2.0};
  EXPECT_DOUBLE_EQ(percentile(v, 0.0), 1.0);
  EXPECT_DOUBLE_EQ(percentile(v, 1.0), 4.0);
  EXPECT_DOUBLE_EQ(percentile(v, 0.5), 2.5);
  EXPECT_DOUBLE_EQ(percentile({}, 0.5), 0.0);
}

TEST(Metrics, DerivativeSkipsNonPositiveSteps)
{
  const auto d = derivative({0.0, 0.1, 0.1, 0.3}, {0.0, 0.1, 0.5, 0.3});
  ASSERT_EQ(d.size(), 2u);
  EXPECT_NEAR(d[0], 1.0, 1e-12);
  EXPECT_NEAR(d[1], -1.0, 1e-12);
  EXPECT_TRUE(derivative({0.0}, {1.0}).empty());
}

TEST(Metrics, PathLength)
{
  EXPECT_DOUBLE_EQ(pathLength({{0, 0, 0}, {3, 4, 0}, {3, 5, 0}}), 6.0);
  EXPECT_DOUBLE_EQ(pathLength({}), 0.0);
}

TEST(Metrics, SummaryOfConstantSpeedRun)
{
  // 2 s at 0.5 m/s along x, constant command: zero acceleration and jerk.
  std::vector<Sample> samples;
  for (int i = 0; i <= 40; ++i) {
    Sample s;
    s.t = 0.05 * i;
    s.pose = mpc_core::StateVec(0.5 * s.t, 0.0, 0.0);
    s.v_cmd = 0.5;
    s.cross_track = 0.01;
    s.clearance = 1.0 - 0.01 * i;
    samples.push_back(s);
  }
  // Threshold between two samples (0.65 itself is not exactly representable).
  const RunSummary r = computeSummary(samples, {1.0, 2.0, 3.0}, 0.645);
  EXPECT_NEAR(r.duration, 2.0, 1e-12);
  EXPECT_NEAR(r.path_length, 1.0, 1e-12);
  EXPECT_NEAR(r.mean_linear_speed, 0.5, 1e-12);
  EXPECT_NEAR(r.cross_track.mean, 0.01, 1e-12);
  EXPECT_NEAR(r.min_clearance, 0.6, 1e-12);
  EXPECT_EQ(r.collision_samples, 5u);  // clearance 0.64 ... 0.60
  EXPECT_NEAR(r.rms_linear_accel, 0.0, 1e-12);
  EXPECT_NEAR(r.rms_linear_jerk, 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(r.compute_time_ms.mean, 2.0);
}

TEST(Metrics, CsvRowsMatchHeaders)
{
  EXPECT_EQ(columns(summaryCsvHeader()), columns(summaryCsvRow(RunSummary{})));
  EXPECT_EQ(columns(samplesCsvHeader()), columns(sampleCsvRow(Sample{})));
}
