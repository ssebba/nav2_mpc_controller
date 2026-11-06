#ifndef MPC_CORE__MPC_SOLVER_HPP_
#define MPC_CORE__MPC_SOLVER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "mpc_core/types.hpp"

namespace mpc_core
{

/**
 * @brief Interface shared by every control back-end.
 *
 * The Nav2 plugin only talks to this interface, so the formulations can be
 * swapped through a ROS parameter and benchmarked against each other.
 */
class MpcSolver
{
public:
  virtual ~MpcSolver() = default;

  // May be called again at runtime (e.g. new weights or speed limit); keeps the warm start
  // unless the problem dimensions change.
  virtual void configure(const MpcConfig & config) = 0;

  /**
   * @param x0 Current robot state
   * @param u_prev Last applied command (rate constraints and increment cost)
   * @param reference N+1 reference states and N feed-forward commands
   * @param obstacles Obstacle points in the same frame as x0
   */
  virtual MpcSolution solve(
    const StateVec & x0, const ControlVec & u_prev, const Reference & reference,
    const ObstaclePoints & obstacles) = 0;

  // Drops any warm-start information (called when a new path is received).
  virtual void reset() = 0;

  virtual std::string name() const = 0;
};

// Creates a solver by name; throws std::invalid_argument for unknown names.
std::unique_ptr<MpcSolver> makeSolver(const std::string & type);

// Names accepted by makeSolver().
std::vector<std::string> availableSolvers();

}  // namespace mpc_core

#endif  // MPC_CORE__MPC_SOLVER_HPP_
