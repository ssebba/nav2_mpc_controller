#include "mpc_core/unicycle_model.hpp"

#include <cmath>
#include <numbers>

namespace mpc_core
{

StateVec UnicycleModel::step(const StateVec & x, const ControlVec & u, double dt)
{
  StateVec next;
  next.x() = x.x() + dt * u.x() * std::cos(x.z());
  next.y() = x.y() + dt * u.x() * std::sin(x.z());
  // Yaw is intentionally NOT normalized: keeping it continuous avoids
  // 2*pi jumps when comparing against an unwrapped reference.
  next.z() = x.z() + dt * u.y();
  return next;
}

StateTrajectory UnicycleModel::rollout(
  const StateVec & x0, const ControlTrajectory & controls, double dt)
{
  StateTrajectory states;
  states.reserve(controls.size() + 1);
  states.push_back(x0);
  for (const auto & u : controls) {
    states.push_back(step(states.back(), u, dt));
  }
  return states;
}

UnicycleModel::MatA UnicycleModel::jacobianState(
  const StateVec & x_bar, const ControlVec & u_bar, double dt)
{
  MatA a = MatA::Identity();
  a(0, 2) = -dt * u_bar.x() * std::sin(x_bar.z());
  a(1, 2) = dt * u_bar.x() * std::cos(x_bar.z());
  return a;
}

UnicycleModel::MatB UnicycleModel::jacobianControl(const StateVec & x_bar, double dt)
{
  MatB b = MatB::Zero();
  b(0, 0) = dt * std::cos(x_bar.z());
  b(1, 0) = dt * std::sin(x_bar.z());
  b(2, 1) = dt;
  return b;
}

double normalizeAngle(double angle)
{
  constexpr double kPi = std::numbers::pi;
  angle = std::fmod(angle + kPi, 2.0 * kPi);
  if (angle <= 0.0) {
    angle += 2.0 * kPi;
  }
  return angle - kPi;
}

}  // namespace mpc_core
