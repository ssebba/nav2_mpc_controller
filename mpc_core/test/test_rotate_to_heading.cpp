#include <cmath>

#include <gtest/gtest.h>

#include "mpc_core/rotate_to_heading.hpp"
#include "mpc_core/unicycle_model.hpp"

using mpc_core::ControlVec;
using mpc_core::Limits;
using mpc_core::rotateInPlaceCommand;

TEST(RotateInPlace, TurnsTowardsTheTarget)
{
  const Limits lim;
  EXPECT_GT(rotateInPlaceCommand(1.0, ControlVec::Zero(), lim, 0.1).y(), 0.0);
  EXPECT_LT(rotateInPlaceCommand(-1.0, ControlVec::Zero(), lim, 0.1).y(), 0.0);
}

TEST(RotateInPlace, RespectsLimits)
{
  Limits lim;
  lim.w_max = 1.0;
  lim.alpha_max = 2.0;
  // Already at w_max, huge error: stays capped.
  EXPECT_NEAR(rotateInPlaceCommand(3.0, ControlVec(0.0, 1.0), lim, 0.1).y(), 1.0, 1e-12);
  // From rest: limited by the angular acceleration.
  EXPECT_NEAR(rotateInPlaceCommand(3.0, ControlVec::Zero(), lim, 0.1).y(), 0.2, 1e-12);
}

TEST(RotateInPlace, BrakesLinearVelocity)
{
  Limits lim;
  lim.a_max = 1.0;
  EXPECT_NEAR(rotateInPlaceCommand(1.0, ControlVec(0.25, 0.0), lim, 0.1).x(), 0.15, 1e-12);
  EXPECT_NEAR(rotateInPlaceCommand(1.0, ControlVec(0.05, 0.0), lim, 0.1).x(), 0.0, 1e-12);
}

TEST(RotateInPlace, ConvergesWithoutOvershoot)
{
  const Limits lim;
  const double dt = 0.05;
  double yaw = 0.0;
  const double target = 2.5;
  ControlVec u = ControlVec::Zero();
  double max_yaw = 0.0;
  for (int i = 0; i < 400; ++i) {
    u = rotateInPlaceCommand(mpc_core::normalizeAngle(target - yaw), u, lim, dt);
    yaw += u.y() * dt;
    max_yaw = std::max(max_yaw, yaw);
  }
  EXPECT_NEAR(yaw, target, 1e-2);
  EXPECT_LT(max_yaw, target + 0.05);
}
