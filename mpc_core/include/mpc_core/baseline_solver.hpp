#ifndef MPC_CORE__BASELINE_SOLVER_HPP_
#define MPC_CORE__BASELINE_SOLVER_HPP_

#include <string>

#include "mpc_core/mpc_solver.hpp"

namespace mpc_core
{

/**
 * @brief Non-optimizing baseline: Kanayama trajectory-tracking feedback law.
 *
 * At every step of the horizon it applies the Kanayama law on the reference,
 * saturates the command (box + rate limits) and rolls the model forward.
 * It ignores obstacles. Used as a benchmark baseline and as the initial guess
 * of the SQP solver.
 *
 * Reference: Kanayama et al., "A stable tracking control method for an
 * autonomous mobile robot", ICRA 1990.
 */
class BaselineSolver : public MpcSolver
{
public:
  void configure(const MpcConfig & config) override;
  MpcSolution solve(
    const StateVec & x0, const ControlVec & u_prev, const Reference & reference,
    const ObstaclePoints & obstacles) override;
  void reset() override {}
  std::string name() const override { return "baseline"; }

private:
  MpcConfig config_;
  // Kanayama gains.
  double k_x_{1.0};
  double k_y_{6.0};
  double k_yaw_{3.0};
};

}  // namespace mpc_core

#endif  // MPC_CORE__BASELINE_SOLVER_HPP_
