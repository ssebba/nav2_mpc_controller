#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "mpc_core/baseline_solver.hpp"
#include "mpc_core/linearized_mpc_solver.hpp"
#include "mpc_core/mpc_solver.hpp"

namespace mpc_core
{

std::unique_ptr<MpcSolver> makeSolver(const std::string & type)
{
  if (type == "baseline") {
    return std::make_unique<BaselineSolver>();
  }
  if (type == "ltv_mpc") {
    return std::make_unique<LinearizedMpcSolver>(LinearizedMpcSolver::Linearization::kReference);
  }
  if (type == "nmpc_sqp") {
    return std::make_unique<LinearizedMpcSolver>(
      LinearizedMpcSolver::Linearization::kPreviousSolution);
  }
  throw std::invalid_argument("Unknown MPC solver type: '" + type + "'");
}

std::vector<std::string> availableSolvers() { return {"baseline", "ltv_mpc", "nmpc_sqp"}; }

}  // namespace mpc_core
