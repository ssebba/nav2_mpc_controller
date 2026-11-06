#include "mpc_core/baseline_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include "mpc_core/tracking_qp.hpp"
#include "mpc_core/unicycle_model.hpp"

namespace mpc_core
{

void BaselineSolver::configure(const MpcConfig & config) { config_ = config; }

MpcSolution BaselineSolver::solve(
  const StateVec & x0, const ControlVec & u_prev, const Reference & reference,
  const ObstaclePoints & /*obstacles*/)
{
  const auto t_start = std::chrono::steady_clock::now();
  const auto n = static_cast<std::size_t>(config_.horizon);
  if (reference.states.size() != n + 1 || reference.controls.size() != n) {
    throw std::invalid_argument("BaselineSolver: reference does not match the horizon");
  }

  MpcSolution sol;
  sol.controls.reserve(n);
  sol.predicted_states.reserve(n + 1);
  sol.predicted_states.push_back(x0);

  StateVec x = x0;
  ControlVec u_last = u_prev;
  for (std::size_t k = 0; k < n; ++k) {
    const StateVec & r = reference.states[k + 1];
    // Tracking error expressed in the robot frame.
    const double c = std::cos(x.z());
    const double s = std::sin(x.z());
    const double ex = c * (r.x() - x.x()) + s * (r.y() - x.y());
    const double ey = -s * (r.x() - x.x()) + c * (r.y() - x.y());
    const double eyaw = normalizeAngle(r.z() - x.z());

    const double v_r = reference.controls[k].x();
    const double w_r = reference.controls[k].y();

    ControlVec u;
    u.x() = v_r * std::cos(eyaw) + k_x_ * ex;
    u.y() = w_r + v_r * (k_y_ * ey + k_yaw_ * std::sin(eyaw));
    // Box and rate limits (the first command over the control period).
    ControlVec lo;
    ControlVec hi;
    commandBounds(config_, u_last, k == 0 ? config_.firstStepPeriod() : config_.dt, lo, hi);
    u = u.cwiseMax(lo).cwiseMin(hi);

    x = UnicycleModel::step(x, u, config_.dt);
    sol.controls.push_back(u);
    sol.predicted_states.push_back(x);
    u_last = u;
  }

  sol.status = SolveStatus::kSuccess;
  sol.solve_time_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_start).count();
  return sol;
}

}  // namespace mpc_core
