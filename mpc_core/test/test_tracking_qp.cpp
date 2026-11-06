#include <cmath>
#include <random>
#include <stdexcept>

#include <gtest/gtest.h>
#include <Eigen/Eigenvalues>

#include "mpc_core/obstacle_constraints.hpp"
#include "mpc_core/reference_builder.hpp"
#include "mpc_core/tracking_qp.hpp"
#include "mpc_core/unicycle_model.hpp"
#include "test_utils.hpp"

using namespace mpc_core;  // NOLINT(google-build-using-namespace)

namespace
{

struct Fixture
{
  MpcConfig config;
  StateVec x0{0.05, -0.1, 0.2};
  ControlVec u_prev{0.1, 0.05};
  Reference reference;
  StateTrajectory x_nominal;
  ControlTrajectory u_nominal;

  explicit Fixture(int horizon = 8)
  {
    config.horizon = horizon;
    reference = ReferenceBuilder::build(
      test::straightPath(3.0, 0.05), x0, config.v_ref, config.dt, config.horizon);
    u_nominal.assign(static_cast<std::size_t>(horizon), ControlVec(0.2, 0.1));
    x_nominal = UnicycleModel::rollout(x0, u_nominal, config.dt);
  }
};

// Dense symmetric matrix from the upper triangular storage used by OSQP.
Eigen::MatrixXd symmetricHessian(const Eigen::SparseMatrix<double> & upper)
{
  const Eigen::MatrixXd p = Eigen::MatrixXd(upper);
  Eigen::MatrixXd full = p + p.transpose();
  full.diagonal() = p.diagonal();
  return full;
}

// Cost of the MPC written term by term, including the constant terms dropped in the QP.
double referenceCost(
  const MpcConfig & cfg, const Reference & ref, const ControlVec & u_prev,
  const StateTrajectory & x, const ControlTrajectory & u, const Eigen::VectorXd & s)
{
  const auto & w = cfg.weights;
  const int n = cfg.horizon;
  double j = 0.0;
  for (int k = 1; k <= n; ++k) {
    const auto ks = static_cast<std::size_t>(k);
    const Eigen::Vector3d & q = (k == n) ? w.q_terminal : w.q;
    const double scale = ref.lateral_weight_scale.empty() ? 1.0 : ref.lateral_weight_scale[ks];
    const StateVec e = x[k] - ref.states[k];
    // Errors along and across the reference heading, weighted separately.
    const double yaw = ref.states[ks].z();
    const double along = std::cos(yaw) * e.x() + std::sin(yaw) * e.y();
    const double across = -std::sin(yaw) * e.x() + std::cos(yaw) * e.y();
    if (scale < 1.0) {
      const double q_along =
        q.x() * std::pow(std::cos(yaw), 2) + q.y() * std::pow(std::sin(yaw), 2);
      const double q_across =
        q.x() * std::pow(std::sin(yaw), 2) + q.y() * std::pow(std::cos(yaw), 2);
      j += 0.5 * (q_along * along * along + scale * q_across * across * across +
                  scale * q.z() * e.z() * e.z());
    } else {
      j += 0.5 * e.dot(q.cwiseProduct(e));
    }
  }
  for (int k = 0; k < n; ++k) {
    const ControlVec e = u[k] - ref.controls[k];
    j += 0.5 * e.dot(w.r.cwiseProduct(e));
    const ControlVec du = u[k] - (k == 0 ? u_prev : u[k - 1]);
    j += 0.5 * du.dot(w.r_delta.cwiseProduct(du));
  }
  for (int k = 0; k < n; ++k) {
    j += cfg.obstacles.slack_linear_weight * s(k) +
         0.5 * cfg.obstacles.slack_quadratic_weight * s(k) * s(k);
  }
  return j;
}

}  // namespace

TEST(TrackingQp, Dimensions)
{
  const TrackingQp qp(10, 2);
  EXPECT_EQ(qp.numVariables(), 3 * 11 + 2 * 10 + 10);
  EXPECT_EQ(qp.numConstraints(), 3 * 11 + 2 * 10 + 2 * 9 + 2 * 10 + 10);
  EXPECT_EQ(qp.stateIndex(0), 0);
  EXPECT_EQ(qp.controlIndex(0), 33);
  EXPECT_EQ(qp.slackIndex(1), 53);
  EXPECT_THROW(TrackingQp(0, 1), std::invalid_argument);
}

TEST(TrackingQp, HessianIsUpperTriangularAndPositiveSemidefinite)
{
  Fixture f;
  const TrackingQp qp(f.config.horizon, 2);
  QpData data;
  qp.build(f.config, f.x0, f.u_prev, f.reference, f.x_nominal, f.u_nominal, {}, data);
  for (int col = 0; col < data.hessian.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(data.hessian, col); it; ++it) {
      EXPECT_LE(it.row(), it.col());
    }
  }
  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(symmetricHessian(data.hessian));
  EXPECT_GE(eig.eigenvalues().minCoeff(), -1e-9);
}

// The sparsity pattern must not depend on the numerical values, otherwise OSQP would
// need a new setup at every control cycle.
TEST(TrackingQp, SparsityPatternIsConstant)
{
  Fixture f;
  const TrackingQp qp(f.config.horizon, 2);
  QpData a;
  QpData b;
  qp.build(f.config, f.x0, f.u_prev, f.reference, f.x_nominal, f.u_nominal, {}, a);

  // Standing still (v = 0 zeroes some Jacobian entries) and with active obstacles.
  const ControlTrajectory u_zero(static_cast<std::size_t>(f.config.horizon), ControlVec::Zero());
  const StateTrajectory x_zero = UnicycleModel::rollout(f.x0, u_zero, f.config.dt);
  const auto obstacles =
    buildObstacleConstraints(x_zero, x_zero, {{0.3, 0.2}, {0.2, -0.3}}, f.config.obstacles);
  qp.build(f.config, f.x0, f.u_prev, f.reference, x_zero, u_zero, obstacles, b);

  ASSERT_EQ(a.constraints.nonZeros(), b.constraints.nonZeros());
  ASSERT_EQ(a.hessian.nonZeros(), b.hessian.nonZeros());
  for (int i = 0; i <= a.constraints.outerSize(); ++i) {
    ASSERT_EQ(a.constraints.outerIndexPtr()[i], b.constraints.outerIndexPtr()[i]);
  }
  for (int i = 0; i < a.constraints.nonZeros(); ++i) {
    ASSERT_EQ(a.constraints.innerIndexPtr()[i], b.constraints.innerIndexPtr()[i]);
  }
}

// Linearizing around a dynamically consistent trajectory, that trajectory satisfies the
// equality constraints exactly.
TEST(TrackingQp, NominalTrajectorySatisfiesDynamics)
{
  Fixture f;
  const TrackingQp qp(f.config.horizon, 0);
  QpData data;
  qp.build(f.config, f.x0, f.u_prev, f.reference, f.x_nominal, f.u_nominal, {}, data);
  const Eigen::VectorXd az = data.constraints * qp.pack(f.x_nominal, f.u_nominal);
  const int dyn_rows = 3 * (f.config.horizon + 1);
  for (int r = 0; r < dyn_rows; ++r) {
    EXPECT_NEAR(az(r), data.lower(r), 1e-12) << "row " << r;
    EXPECT_DOUBLE_EQ(data.lower(r), data.upper(r));
  }
}

class CostDefinition : public ::testing::TestWithParam<bool>
{
};

TEST_P(CostDefinition, MatchesTermByTermDefinition)
{
  Fixture f;
  if (GetParam()) {
    // Per-sample lateral weight scales, zero included (a fully relaxed sample), on a curved
    // reference so that the along/across decomposition differs from the x/y one.
    f.config.weights.q = Eigen::Vector3d(8.0, 12.0, 1.5);
    f.reference.lateral_weight_scale = {1.0, 0.1, 0.0, 1.0, 0.5, 0.1, 1.0, 0.0, 0.3};
    for (std::size_t k = 0; k < f.reference.states.size(); ++k) {
      f.reference.states[k].z() = 0.3 * static_cast<double>(k) - 1.0;
    }
  }
  const TrackingQp qp(f.config.horizon, 1);
  QpData data;
  qp.build(f.config, f.x0, f.u_prev, f.reference, f.x_nominal, f.u_nominal, {}, data);
  const Eigen::MatrixXd p = symmetricHessian(data.hessian);

  // The QP drops constant terms: the difference must be the same for any z.
  std::mt19937 rng(42);
  std::normal_distribution<double> noise(0.0, 0.3);
  double offset = 0.0;
  for (int trial = 0; trial < 5; ++trial) {
    StateTrajectory x = f.x_nominal;
    ControlTrajectory u = f.u_nominal;
    for (auto & xi : x) {
      xi += StateVec(noise(rng), noise(rng), noise(rng));
    }
    for (auto & ui : u) {
      ui += ControlVec(noise(rng), noise(rng));
    }
    Eigen::VectorXd z = qp.pack(x, u);
    Eigen::VectorXd s(f.config.horizon);
    for (int k = 0; k < f.config.horizon; ++k) {
      s(k) = std::abs(noise(rng));
      z(qp.slackIndex(k + 1)) = s(k);
    }
    const double qp_cost = 0.5 * z.dot(p * z) + data.gradient.dot(z);
    const double diff = referenceCost(f.config, f.reference, f.u_prev, x, u, s) - qp_cost;
    if (trial == 0) {
      offset = diff;
    } else {
      EXPECT_NEAR(diff, offset, 1e-9);
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
  TrackingQp, CostDefinition, ::testing::Values(false, true),
  [](const ::testing::TestParamInfo<bool> & param_info) {
    return param_info.param ? "WithLateralWeightScale" : "Plain";
  });

TEST(TrackingQp, ZeroLateralWeightScaleKeepsTheSparsityPattern)
{
  Fixture f;
  const TrackingQp qp(f.config.horizon, 1);
  QpData a;
  QpData b;
  qp.build(f.config, f.x0, f.u_prev, f.reference, f.x_nominal, f.u_nominal, {}, a);
  f.reference.lateral_weight_scale.assign(static_cast<std::size_t>(f.config.horizon + 1), 0.0);
  qp.build(f.config, f.x0, f.u_prev, f.reference, f.x_nominal, f.u_nominal, {}, b);
  ASSERT_EQ(a.hessian.nonZeros(), b.hessian.nonZeros());
  for (int i = 0; i < a.hessian.nonZeros(); ++i) {
    ASSERT_EQ(a.hessian.innerIndexPtr()[i], b.hessian.innerIndexPtr()[i]);
  }
  f.reference.lateral_weight_scale.pop_back();
  EXPECT_THROW(
    qp.build(f.config, f.x0, f.u_prev, f.reference, f.x_nominal, f.u_nominal, {}, b),
    std::invalid_argument);
}

TEST(TrackingQp, FirstCommandRespectsRateLimitAndBoxWins)
{
  Fixture f;
  const TrackingQp qp(f.config.horizon, 0);
  const int row_u0 = 3 * (f.config.horizon + 1);
  QpData data;

  qp.build(f.config, f.x0, f.u_prev, f.reference, f.x_nominal, f.u_nominal, {}, data);
  const double dv = f.config.limits.a_max * f.config.dt;
  EXPECT_NEAR(data.lower(row_u0), std::max(f.config.limits.v_min, f.u_prev.x() - dv), 1e-12);
  EXPECT_NEAR(data.upper(row_u0), std::min(f.config.limits.v_max, f.u_prev.x() + dv), 1e-12);

  // Previous command far above a newly reduced speed limit: the box wins.
  const ControlVec fast(0.9, 0.0);
  qp.build(f.config, f.x0, fast, f.reference, f.x_nominal, f.u_nominal, {}, data);
  EXPECT_DOUBLE_EQ(data.lower(row_u0), f.config.limits.v_max);
  EXPECT_DOUBLE_EQ(data.upper(row_u0), f.config.limits.v_max);
}

TEST(TrackingQp, PackUnpackRoundTrip)
{
  Fixture f;
  const TrackingQp qp(f.config.horizon, 2);
  StateTrajectory x;
  ControlTrajectory u;
  Eigen::VectorXd s;
  qp.unpack(qp.pack(f.x_nominal, f.u_nominal), x, u, s);
  for (std::size_t k = 0; k < x.size(); ++k) {
    EXPECT_TRUE(x[k].isApprox(f.x_nominal[k]));
  }
  for (std::size_t k = 0; k < u.size(); ++k) {
    EXPECT_TRUE(u[k].isApprox(f.u_nominal[k]));
  }
  EXPECT_DOUBLE_EQ(s.cwiseAbs().maxCoeff(), 0.0);
}
