#ifndef MPC_CORE__LINEARIZED_MPC_SOLVER_HPP_
#define MPC_CORE__LINEARIZED_MPC_SOLVER_HPP_

#include <memory>
#include <string>

#include "mpc_core/baseline_solver.hpp"
#include "mpc_core/mpc_solver.hpp"
#include "mpc_core/osqp_solver.hpp"
#include "mpc_core/tracking_qp.hpp"

namespace mpc_core
{

/**
 * @brief MPC solved as a sequence of sparse QPs (OSQP), one family, two strategies.
 *
 * - kReference ("ltv_mpc"): classic linear time-varying MPC. The model is linearized
 *   once around the reference trajectory. Cheap and predictable, but the linearization
 *   degrades when the robot is far from the reference (large heading errors).
 *
 * - kPreviousSolution ("nmpc_sqp"): nonlinear MPC by Sequential Quadratic Programming.
 *   The model is linearized around the previous solution shifted by one step
 *   (initial guess from the Kanayama baseline) and the QP is re-solved up to
 *   `sqp_iterations` times. With one iteration this is the Real-Time Iteration
 *   scheme (Diehl et al., 2005).
 *
 * Both rebuild the obstacle half-planes around the current linearization point.
 * If a QP fails, the previous solution is shifted and replayed for at most
 * `max_consecutive_failures` cycles.
 */
class LinearizedMpcSolver : public MpcSolver
{
public:
  enum class Linearization
  {
    kReference,
    kPreviousSolution,
  };

  explicit LinearizedMpcSolver(Linearization linearization);

  void configure(const MpcConfig & config) override;
  MpcSolution solve(
    const StateVec & x0, const ControlVec & u_prev, const Reference & reference,
    const ObstaclePoints & obstacles) override;
  void reset() override;
  std::string name() const override;

  // Exposed for tests: number of OSQP setups (should stay at 1 while N and M are unchanged).
  int setupCount() const { return qp_solver_.setupCount(); }

private:
  // Linearization trajectory of the first iteration.
  void nominalTrajectory(
    const StateVec & x0, const ControlVec & u_prev, const Reference & reference,
    StateTrajectory & x_nominal, ControlTrajectory & u_nominal);
  // previous_controls_ shifted by one step (last command repeated).
  ControlTrajectory shiftedPreviousControls() const;

  Linearization linearization_;
  MpcConfig config_;
  std::unique_ptr<TrackingQp> qp_;
  OsqpSolver qp_solver_;
  QpData qp_data_;
  BaselineSolver initial_guess_;
  // Last solution (unshifted); empty when no warm start is available.
  ControlTrajectory previous_controls_;
  int consecutive_failures_{0};
};

}  // namespace mpc_core

#endif  // MPC_CORE__LINEARIZED_MPC_SOLVER_HPP_
