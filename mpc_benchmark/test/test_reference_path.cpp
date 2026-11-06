#include <cmath>
#include <string>

#include <gtest/gtest.h>

#include "mpc_benchmark/metrics.hpp"
#include "mpc_benchmark/scenario_runner.hpp"

using mpc_benchmark::makeReferencePath;

class ReferencePathShapes : public ::testing::TestWithParam<std::string>
{
};

// The robot starts at rest at the origin heading along +x: every reference path must start
// there with the same heading, otherwise the first seconds of a run measure the start-up.
TEST_P(ReferencePathShapes, StartsAtTheOriginAlongX)
{
  scenario_runner::Params params;
  params.path.shape = GetParam();
  const auto path = makeReferencePath(params);
  ASSERT_GT(path.size(), 10u);
  EXPECT_NEAR(path.front().x(), 0.0, 1e-9);
  EXPECT_NEAR(path.front().y(), 0.0, 1e-9);
  EXPECT_NEAR(path.front().z(), 0.0, 1e-9);
  // The tangent of the first segment points along +x too.
  const auto d = path[1].head<2>() - path[0].head<2>();
  EXPECT_NEAR(std::atan2(d.y(), d.x()), 0.0, 1e-3);
}

INSTANTIATE_TEST_SUITE_P(
  Shapes, ReferencePathShapes, ::testing::Values("figure_eight", "sine", "straight"),
  [](const auto & param_info) { return param_info.param; });

TEST(ReferencePath, FigureEightLength)
{
  scenario_runner::Params params;
  params.path.shape = "figure_eight";
  params.path.size = 1.5;
  // Lemniscate of Gerono with a = 1.5: length ~ 6.1 a.
  EXPECT_NEAR(mpc_benchmark::pathLength(makeReferencePath(params)), 9.15, 0.1);
}
