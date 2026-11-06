#ifndef MPC_CORE__CLOSED_LOOP_HPP_
#define MPC_CORE__CLOSED_LOOP_HPP_

#include <vector>

#include "mpc_core/mpc_solver.hpp"
#include "mpc_core/types.hpp"

namespace mpc_core
{

struct ClosedLoopResult
{
  StateTrajectory states;      // x_1 ... x_T
  ControlTrajectory controls;  // applied commands u_0 ... u_{T-1}
  std::vector<MpcSolution> solutions;
  std::vector<double> cross_track;  // distance from the path after each step
  double max_cross_track{0.0};
  double rms_cross_track{0.0};
  // Fraction of the path length covered (projection of the final state).
  double progress{0.0};
  bool all_ok{true};
  bool reached_end{false};
};

/**
 * @brief Receding-horizon simulation on the nominal unicycle model (no ROS, no noise).
 *
 * At every cycle the path is pruned up to the robot (as the Nav2 path handler does), the
 * reference is rebuilt, the solver is called and only the first command is applied.
 * Stops early when the solver fails or when the robot has reached the last path segment,
 * is within `goal_tolerance` of the end of the path and almost still.
 */
ClosedLoopResult runClosedLoop(
  MpcSolver & solver, const MpcConfig & config, const StateTrajectory & path, const StateVec & x0,
  int max_cycles, const ObstaclePoints & obstacles = {}, double goal_tolerance = 0.0);

}  // namespace mpc_core

#endif  // MPC_CORE__CLOSED_LOOP_HPP_
