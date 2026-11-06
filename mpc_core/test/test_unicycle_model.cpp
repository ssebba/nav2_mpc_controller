#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

#include "mpc_core/unicycle_model.hpp"

using mpc_core::ControlTrajectory;
using mpc_core::ControlVec;
using mpc_core::normalizeAngle;
using mpc_core::StateVec;
using mpc_core::UnicycleModel;

TEST(UnicycleModel, StraightLine)
{
  const StateVec next = UnicycleModel::step(StateVec::Zero(), ControlVec(1.0, 0.0), 0.1);
  EXPECT_NEAR(next.x(), 0.1, 1e-12);
  EXPECT_NEAR(next.y(), 0.0, 1e-12);
  EXPECT_NEAR(next.z(), 0.0, 1e-12);
}

TEST(UnicycleModel, PureRotation)
{
  const StateVec next = UnicycleModel::step(StateVec(1.0, 2.0, 0.5), ControlVec(0.0, 1.0), 0.1);
  EXPECT_NEAR(next.x(), 1.0, 1e-12);
  EXPECT_NEAR(next.y(), 2.0, 1e-12);
  EXPECT_NEAR(next.z(), 0.6, 1e-12);
}

TEST(UnicycleModel, YawIsNotWrapped)
{
  const StateVec next =
    UnicycleModel::step(StateVec(0.0, 0.0, std::numbers::pi - 0.01), ControlVec(0.0, 1.0), 0.1);
  EXPECT_GT(next.z(), std::numbers::pi);
}

TEST(UnicycleModel, RolloutLength)
{
  const ControlTrajectory u(10, ControlVec(0.5, 0.1));
  EXPECT_EQ(UnicycleModel::rollout(StateVec::Zero(), u, 0.1).size(), 11u);
}

// The analytic Jacobians must match a central finite-difference approximation.
TEST(UnicycleModel, JacobiansMatchFiniteDifferences)
{
  const StateVec x(0.3, -0.2, 0.8);
  const ControlVec u(0.4, -0.6);
  const double dt = 0.1;
  const double eps = 1e-6;

  const auto a = UnicycleModel::jacobianState(x, u, dt);
  const auto b = UnicycleModel::jacobianControl(x, dt);

  for (int i = 0; i < 3; ++i) {
    StateVec dx = StateVec::Zero();
    dx(i) = eps;
    const StateVec col =
      (UnicycleModel::step(x + dx, u, dt) - UnicycleModel::step(x - dx, u, dt)) / (2 * eps);
    EXPECT_TRUE(col.isApprox(a.col(i), 1e-6)) << "A column " << i;
  }
  for (int i = 0; i < 2; ++i) {
    ControlVec du = ControlVec::Zero();
    du(i) = eps;
    const StateVec col =
      (UnicycleModel::step(x, u + du, dt) - UnicycleModel::step(x, u - du, dt)) / (2 * eps);
    EXPECT_TRUE(col.isApprox(b.col(i), 1e-6)) << "B column " << i;
  }
}

TEST(NormalizeAngle, WrapsToPlusMinusPi)
{
  constexpr double kPi = std::numbers::pi;
  EXPECT_NEAR(normalizeAngle(0.0), 0.0, 1e-12);
  EXPECT_NEAR(normalizeAngle(3.0 * kPi), kPi, 1e-12);
  EXPECT_NEAR(normalizeAngle(-kPi), kPi, 1e-12);
  EXPECT_NEAR(normalizeAngle(-3.0 * kPi / 2.0), kPi / 2.0, 1e-12);
  EXPECT_NEAR(normalizeAngle(2.0 * kPi + 0.1), 0.1, 1e-12);
}
