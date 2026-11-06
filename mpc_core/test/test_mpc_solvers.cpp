#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "mpc_core/linearized_mpc_solver.hpp"
#include "mpc_core/mpc_solver.hpp"
#include "mpc_core/obstacle_constraints.hpp"
#include "mpc_core/reference_builder.hpp"
#include "mpc_core/unicycle_model.hpp"
#include "test_utils.hpp"

using namespace mpc_core;  // NOLINT(google-build-using-namespace)

namespace
{

MpcConfig defaultConfig()
{
  MpcConfig cfg;
  cfg.horizon = 20;
  cfg.dt = 0.1;
  cfg.v_ref = 0.25;
  return cfg;
}

// Path recorded in a Gazebo run (test/data): "x,y" lines, '#' comments.
StateTrajectory recordedPath(const std::string & file)
{
  std::ifstream in(std::string(MPC_CORE_TEST_DATA_DIR) + "/" + file);
  StateTrajectory path;
  std::string line;
  while (std::getline(in, line)) {
    double x = 0.0;
    double y = 0.0;
    if (!line.empty() && line.front() != '#' && std::sscanf(line.c_str(), "%lf,%lf", &x, &y) == 2) {
      path.emplace_back(x, y, 0.0);
    }
  }
  ReferenceBuilder::assignTangentYaw(path);
  return path;
}

std::unique_ptr<MpcSolver> configured(const std::string & name, const MpcConfig & cfg)
{
  auto solver = makeSolver(name);
  solver->configure(cfg);
  return solver;
}

}  // namespace

// ---------------------------------------------------------------------------
// Properties shared by every back-end.
// ---------------------------------------------------------------------------
class AllSolvers : public ::testing::TestWithParam<std::string>
{
};

TEST_P(AllSolvers, AppliedCommandsRespectLimits)
{
  const auto cfg = defaultConfig();
  auto solver = configured(GetParam(), cfg);
  // Start far from the path and badly oriented to drive the commands into saturation.
  const auto res =
    test::runClosedLoop(*solver, cfg, test::straightPath(6.0, 0.05, 0.8), {0.0, 0.0, -1.2}, 120);
  ASSERT_TRUE(res.all_ok);
  ControlVec u_prev = ControlVec::Zero();
  const double tol = 1e-4;
  for (const auto & u : res.controls) {
    EXPECT_LE(u.x(), cfg.limits.v_max + tol);
    EXPECT_GE(u.x(), cfg.limits.v_min - tol);
    EXPECT_LE(std::abs(u.y()), cfg.limits.w_max + tol);
    EXPECT_LE(std::abs(u.x() - u_prev.x()), cfg.limits.a_max * cfg.dt + tol);
    EXPECT_LE(std::abs(u.y() - u_prev.y()), cfg.limits.alpha_max * cfg.dt + tol);
    u_prev = u;
  }
}

TEST_P(AllSolvers, ConvergesToStraightPath)
{
  const auto cfg = defaultConfig();
  auto solver = configured(GetParam(), cfg);
  const auto res =
    test::runClosedLoop(*solver, cfg, test::straightPath(10.0, 0.05), {0.0, 0.4, 0.4}, 200);
  ASSERT_TRUE(res.all_ok);
  EXPECT_NEAR(res.states.back().y(), 0.0, 0.03);
  EXPECT_NEAR(normalizeAngle(res.states.back().z()), 0.0, 0.05);
}

TEST_P(AllSolvers, TracksFigureEight)
{
  const auto cfg = defaultConfig();
  auto solver = configured(GetParam(), cfg);
  const auto path = test::figureEight(1.5, 0.05);
  // ~70% of the lap at 0.25 m/s.
  const auto res = test::runClosedLoop(*solver, cfg, path, path.front(), 300);
  ASSERT_TRUE(res.all_ok);
  EXPECT_LT(res.rms_cross_track, 0.05) << GetParam();
  EXPECT_LT(res.max_cross_track, 0.12) << GetParam();
}

TEST_P(AllSolvers, StopsAtTheEndOfThePath)
{
  const auto cfg = defaultConfig();
  auto solver = configured(GetParam(), cfg);
  const auto res =
    test::runClosedLoop(*solver, cfg, test::straightPath(1.0, 0.05), {0.0, 0.0, 0.0}, 150);
  ASSERT_TRUE(res.all_ok);
  EXPECT_NEAR(res.states.back().x(), 1.0, 0.05);
  EXPECT_NEAR(res.controls.back().x(), 0.0, 0.02);
}

INSTANTIATE_TEST_SUITE_P(
  Solvers, AllSolvers, ::testing::Values("baseline", "ltv_mpc", "nmpc_sqp"),
  [](const auto & param_info) { return param_info.param; });

// ---------------------------------------------------------------------------
// Optimization-based back-ends only.
// ---------------------------------------------------------------------------
class QpSolvers : public ::testing::TestWithParam<std::string>
{
};

TEST_P(QpSolvers, AvoidsObstacleNextToThePath)
{
  auto cfg = defaultConfig();
  cfg.obstacles.safety_distance = 0.3;
  auto solver = configured(GetParam(), cfg);
  // Obstacle 0.22 m from the path: the robot must deviate ~8 cm to keep 0.3 m clearance.
  const ObstaclePoints obstacles{{1.5, 0.22}};
  const auto res = test::runClosedLoop(
    *solver, cfg, test::straightPath(4.0, 0.05), {0.0, 0.0, 0.0}, 160, obstacles);
  ASSERT_TRUE(res.all_ok);
  EXPECT_GT(minimumClearance(res.states, obstacles), 0.3 - 0.02) << GetParam();
  EXPECT_GT(res.states.back().x(), 3.0) << "the robot should get past the obstacle";
  EXPECT_LT(std::abs(res.states.back().y()), 0.03) << "and come back to the path";
}

// An obstacle almost on the path requires a large deviation. The SQP solver re-linearizes
// around its own (moving) solution and escapes; the LTV solver, linearized on the reference
// heading, stalls in front of it (documented limitation, see the README benchmark).
TEST(NmpcSqp, EscapesObstacleOnThePath)
{
  auto cfg = defaultConfig();
  cfg.obstacles.safety_distance = 0.3;
  auto solver = configured("nmpc_sqp", cfg);
  const ObstaclePoints obstacles{{1.5, 0.1}};
  const auto res = test::runClosedLoop(
    *solver, cfg, test::straightPath(4.0, 0.05), {0.0, 0.0, 0.0}, 160, obstacles);
  ASSERT_TRUE(res.all_ok);
  EXPECT_GT(minimumClearance(res.states, obstacles), 0.3 - 0.02);
  EXPECT_GT(res.states.back().x(), 3.0);
}

// A wide obstacle across the path (the face of a 30 cm box, as seen in a costmap): points on
// both sides of the path must not push the robot in opposite directions (deadlock).
TEST(NmpcSqp, EscapesWideObstacleAcrossThePath)
{
  auto cfg = defaultConfig();
  cfg.obstacles.safety_distance = 0.3;
  auto solver = configured("nmpc_sqp", cfg);
  ObstaclePoints box;
  for (double x = 1.35; x <= 1.65 + 1e-9; x += 0.05) {
    for (double y = -0.07; y <= 0.23 + 1e-9; y += 0.05) {
      box.emplace_back(x, y);
    }
  }
  const auto res =
    test::runClosedLoop(*solver, cfg, test::straightPath(4.0, 0.05), {0.0, 0.0, 0.0}, 200, box);
  ASSERT_TRUE(res.all_ok);
  EXPECT_GT(minimumClearance(res.states, box), 0.3 - 0.03);
  EXPECT_GT(res.states.back().x(), 3.0) << "stuck in front of the obstacle";
  // More free space on the right (the box is shifted to the left): pass on the right.
  double min_y = 0.0;
  for (const auto & s : res.states) {
    min_y = std::min(min_y, s.y());
  }
  EXPECT_LT(min_y, -0.3);
}

// Benchmark S3, replayed from a Gazebo run (recorded plan, state and command, 20 Hz loop):
// the planner, blind to a box, drew its plan diagonally through it while the robot was
// beside the box. Tracking that reference across the path pulled the robot against the
// constraints and the optimum became "advance by almost nothing" at every cycle: the robot
// stalled at x = 2.90 until Nav2's progress checker gave up. Reference samples inside the
// safety margin are now tracked along the path only (ObstacleConfig::blocked_reference_weight).
TEST(NmpcSqp, KeepsMovingBesideAnObstacleThePlanCrosses)
{
  auto cfg = defaultConfig();
  cfg.control_period = 0.05;
  cfg.obstacles.safety_distance = 0.3;
  auto solver = configured("nmpc_sqp", cfg);
  ObstaclePoints box;
  for (double x = 2.875; x <= 3.125 + 1e-9; x += 0.05) {
    for (double y = -0.225; y <= 0.075 + 1e-9; y += 0.05) {
      box.emplace_back(x, y);
    }
  }
  box = downsamplePoints(box, 0.1);
  const StateTrajectory path = recordedPath("s3_plan_through_box.csv");
  ASSERT_GT(path.size(), 100u);

  StateVec x(2.8013, -0.5224, -0.1022);
  ControlVec u(0.106, 0.340);
  std::size_t start = 0;
  double min_clearance = std::numeric_limits<double>::infinity();
  for (int cycle = 0; cycle < 120; ++cycle) {  // 6 s
    // The path is pruned up to the robot, as the Nav2 path handler does.
    const StateTrajectory remaining(path.begin() + static_cast<std::ptrdiff_t>(start), path.end());
    start += ReferenceBuilder::closestPoint(remaining, x.head<2>(), 1.0).segment;
    const StateTrajectory pruned(path.begin() + static_cast<std::ptrdiff_t>(start), path.end());
    const Reference ref = ReferenceBuilder::build(pruned, x, cfg.v_ref, cfg.dt, cfg.horizon);
    const MpcSolution sol = solver->solve(x, u, ref, box);
    ASSERT_TRUE(sol.ok());
    u = sol.controls.front();
    x = UnicycleModel::step(x, u, cfg.control_period);
    min_clearance = std::min(min_clearance, minimumClearance({x}, box));
  }
  EXPECT_GT(x.x(), 3.9) << "stalled beside the box";
  EXPECT_GT(min_clearance, 0.3 - 0.03);
}

// A cold start exactly on the boundary of a safety margin (start of the figure eight, an
// obstacle point 0.3 m away). With OSQP 1.0's duality-gap termination criterion enabled this
// first QP did not converge within 4000 iterations (see OsqpSolver).
TEST_P(QpSolvers, ColdStartOnTheSafetyBoundaryConverges)
{
  const auto cfg = defaultConfig();
  auto solver = configured(GetParam(), cfg);
  const auto path = test::figureEight(1.5, 0.05);
  const ObstaclePoints obstacles{{1.06, 1.0}, {-1.06, -1.0}, {0.0, 0.3}};
  const Reference ref = ReferenceBuilder::build(path, path.front(), cfg.v_ref, cfg.dt, cfg.horizon);
  const auto sol = solver->solve(path.front(), ControlVec::Zero(), ref, obstacles);
  EXPECT_EQ(sol.status, SolveStatus::kSuccess);
  EXPECT_LT(sol.qp_iterations, cfg.solver.max_qp_iterations);
}

TEST_P(QpSolvers, ReusesTheOsqpWorkspace)
{
  const auto cfg = defaultConfig();
  LinearizedMpcSolver solver(
    GetParam() == "ltv_mpc" ? LinearizedMpcSolver::Linearization::kReference
                            : LinearizedMpcSolver::Linearization::kPreviousSolution);
  solver.configure(cfg);
  const auto res = test::runClosedLoop(
    solver, cfg, test::straightPath(4.0, 0.05), {0.0, 0.1, 0.0}, 50, {{1.0, 0.6}});
  ASSERT_TRUE(res.all_ok);
  EXPECT_EQ(solver.setupCount(), 1);
  // A runtime re-configuration with the same dimensions keeps the workspace.
  auto cfg2 = cfg;
  cfg2.limits.v_max = 0.1;
  cfg2.weights.q(0) = 30.0;
  solver.configure(cfg2);
  test::runClosedLoop(solver, cfg2, test::straightPath(4.0, 0.05), {0.0, 0.1, 0.0}, 5);
  EXPECT_EQ(solver.setupCount(), 1);
}

TEST_P(QpSolvers, FallsBackToPreviousSolutionThenFails)
{
  auto cfg = defaultConfig();
  auto solver = configured(GetParam(), cfg);
  const auto path = test::straightPath(4.0, 0.05);
  const StateVec x0(0.0, 0.2, 0.0);
  const auto ref = ReferenceBuilder::build(path, x0, cfg.v_ref, cfg.dt, cfg.horizon);
  ASSERT_EQ(solver->solve(x0, ControlVec::Zero(), ref, {}).status, SolveStatus::kSuccess);

  // Starve OSQP so that it cannot converge anymore.
  cfg.solver.max_qp_iterations = 1;
  cfg.solver.eps_abs = 1e-12;
  cfg.solver.eps_rel = 1e-12;
  cfg.solver.max_consecutive_failures = 2;
  solver->configure(cfg);
  const StateVec x1(0.3, -0.3, 0.5);
  const auto ref1 = ReferenceBuilder::build(path, x1, cfg.v_ref, cfg.dt, cfg.horizon);
  EXPECT_EQ(solver->solve(x1, ControlVec::Zero(), ref1, {}).status, SolveStatus::kFallback);
  EXPECT_EQ(solver->solve(x1, ControlVec::Zero(), ref1, {}).status, SolveStatus::kFallback);
  const auto failed = solver->solve(x1, ControlVec::Zero(), ref1, {});
  EXPECT_EQ(failed.status, SolveStatus::kFailure);
  EXPECT_FALSE(failed.ok());
}

TEST_P(QpSolvers, SolveTimeIsCompatibleWithA20HzLoop)
{
  const auto cfg = defaultConfig();
  auto solver = configured(GetParam(), cfg);
  const auto path = test::figureEight(1.5, 0.05);
  const auto res = test::runClosedLoop(*solver, cfg, path, path.front(), 100, {{0.5, 0.8}});
  ASSERT_TRUE(res.all_ok);
  double total = 0.0;
  for (const auto & s : res.solutions) {
    total += s.solve_time_ms;
  }
  // Loose bound to stay robust on slow CI runners; actual figures are in the README.
  EXPECT_LT(total / static_cast<double>(res.solutions.size()), 25.0);
}

INSTANTIATE_TEST_SUITE_P(
  Solvers, QpSolvers, ::testing::Values("ltv_mpc", "nmpc_sqp"),
  [](const auto & param_info) { return param_info.param; });

TEST(SolverFactory, KnownAndUnknownNames)
{
  for (const auto & name : availableSolvers()) {
    EXPECT_EQ(makeSolver(name)->name(), name);
  }
  EXPECT_THROW(makeSolver("does_not_exist"), std::invalid_argument);
}

TEST(LinearizedMpcSolver, RejectsInvalidConfiguration)
{
  auto cfg = defaultConfig();
  cfg.horizon = 1;
  EXPECT_THROW(makeSolver("nmpc_sqp")->configure(cfg), std::invalid_argument);
}
