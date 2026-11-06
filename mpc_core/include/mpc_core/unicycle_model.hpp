#ifndef MPC_CORE__UNICYCLE_MODEL_HPP_
#define MPC_CORE__UNICYCLE_MODEL_HPP_

#include <Eigen/Core>

#include "mpc_core/types.hpp"

namespace mpc_core
{

/**
 * @brief Kinematic unicycle model discretized with forward Euler.
 *
 *   x_{k+1}   = x_k   + dt * v_k * cos(yaw_k)
 *   y_{k+1}   = y_k   + dt * v_k * sin(yaw_k)
 *   yaw_{k+1} = yaw_k + dt * w_k
 *
 * The Jacobians are those of the same discretization, so the linearized
 * prediction used by the MPC is consistent with the nonlinear one.
 */
class UnicycleModel
{
public:
  using MatA = Eigen::Matrix3d;
  using MatB = Eigen::Matrix<double, kStateDim, kControlDim>;

  static StateVec step(const StateVec & x, const ControlVec & u, double dt);

  // Rolls the model forward from x0 applying the given controls; returns N+1 states.
  static StateTrajectory rollout(
    const StateVec & x0, const ControlTrajectory & controls, double dt);

  // Linearization around (x_bar, u_bar): x_{k+1} ~= A x_k + B u_k + c.
  static MatA jacobianState(const StateVec & x_bar, const ControlVec & u_bar, double dt);
  static MatB jacobianControl(const StateVec & x_bar, double dt);
};

// Wraps an angle to (-pi, pi].
double normalizeAngle(double angle);

}  // namespace mpc_core

#endif  // MPC_CORE__UNICYCLE_MODEL_HPP_
