#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "mpc_core/osqp_solver.hpp"

using mpc_core::OsqpSolver;
using mpc_core::QpData;

namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();

// min 0.5 |z|^2 + q^T z   s.t.  z0 + z1 <= 1,  z >= 0
QpData simpleQp(double q0, double q1)
{
  QpData qp;
  qp.hessian.resize(2, 2);
  std::vector<Eigen::Triplet<double>> p{{0, 0, 1.0}, {1, 1, 1.0}};
  qp.hessian.setFromTriplets(p.begin(), p.end());
  qp.gradient = Eigen::Vector2d(q0, q1);
  qp.constraints.resize(3, 2);
  std::vector<Eigen::Triplet<double>> a{{0, 0, 1.0}, {0, 1, 1.0}, {1, 0, 1.0}, {2, 1, 1.0}};
  qp.constraints.setFromTriplets(a.begin(), a.end());
  qp.lower = Eigen::Vector3d(-kInf, 0.0, 0.0);
  qp.upper = Eigen::Vector3d(1.0, kInf, kInf);
  return qp;
}

OsqpSolver::Settings accurate()
{
  OsqpSolver::Settings s;
  s.eps_abs = 1e-7;
  s.eps_rel = 1e-7;
  s.polish = true;
  return s;
}

}  // namespace

TEST(OsqpSolver, SolvesSimpleQp)
{
  OsqpSolver solver(accurate());
  const auto res = solver.solve(simpleQp(-1.0, -1.0));
  ASSERT_TRUE(res.solved) << res.status;
  EXPECT_NEAR(res.primal(0), 0.5, 1e-4);
  EXPECT_NEAR(res.primal(1), 0.5, 1e-4);
}

TEST(OsqpSolver, UpdatesValuesWithoutNewSetup)
{
  OsqpSolver solver(accurate());
  ASSERT_TRUE(solver.solve(simpleQp(-1.0, -1.0)).solved);
  const auto res = solver.solve(simpleQp(-1.0, 0.0));
  ASSERT_TRUE(res.solved);
  EXPECT_NEAR(res.primal(0), 1.0, 1e-4);
  EXPECT_NEAR(res.primal(1), 0.0, 1e-4);
  EXPECT_EQ(solver.setupCount(), 1);
}

TEST(OsqpSolver, NewPatternTriggersSetup)
{
  OsqpSolver solver(accurate());
  ASSERT_TRUE(solver.solve(simpleQp(-1.0, -1.0)).solved);
  QpData qp = simpleQp(-1.0, -1.0);
  // Add an off-diagonal Hessian entry: different pattern.
  std::vector<Eigen::Triplet<double>> p{{0, 0, 1.0}, {0, 1, 0.1}, {1, 1, 1.0}};
  qp.hessian.setFromTriplets(p.begin(), p.end());
  ASSERT_TRUE(solver.solve(qp).solved);
  EXPECT_EQ(solver.setupCount(), 2);
}

TEST(OsqpSolver, ChangingSettingsDropsWorkspace)
{
  OsqpSolver solver(accurate());
  ASSERT_TRUE(solver.solve(simpleQp(-1.0, -1.0)).solved);
  auto s = accurate();
  s.max_iterations = 100;
  solver.setSettings(s);
  ASSERT_TRUE(solver.solve(simpleQp(-1.0, -1.0)).solved);
  EXPECT_EQ(solver.setupCount(), 2);
}

TEST(OsqpSolver, TimeLimitIsReportedAsFailure)
{
  auto s = accurate();
  s.time_limit = 1e-9;
  s.eps_abs = 1e-12;
  s.eps_rel = 1e-12;
  OsqpSolver solver(s);
  const auto res = solver.solve(simpleQp(-1.0, -1.0));
  EXPECT_FALSE(res.solved);
  EXPECT_NE(res.status.find("time"), std::string::npos) << res.status;
}

TEST(OsqpSolver, ReportsInfeasibility)
{
  QpData qp = simpleQp(-1.0, -1.0);
  // z0 + z1 >= 3 while z0, z1 <= 1.
  qp.lower = Eigen::Vector3d(3.0, 0.0, 0.0);
  qp.upper = Eigen::Vector3d(kInf, 1.0, 1.0);
  OsqpSolver solver;
  const auto res = solver.solve(qp);
  EXPECT_FALSE(res.solved);
  EXPECT_FALSE(res.status.empty());
}
