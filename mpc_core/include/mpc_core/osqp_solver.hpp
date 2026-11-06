#ifndef MPC_CORE__OSQP_SOLVER_HPP_
#define MPC_CORE__OSQP_SOLVER_HPP_

#include <memory>
#include <string>

#include <Eigen/Core>

#include "mpc_core/tracking_qp.hpp"

namespace mpc_core
{

/**
 * @brief RAII wrapper around the OSQP 1.x C API.
 *
 * The first solve performs the setup (allocation + KKT factorization). Following
 * solves with the same dimensions and sparsity pattern only update the numerical
 * values, which is what makes OSQP fast inside an MPC loop.
 * The OSQP headers are kept out of this interface (pimpl).
 */
class OsqpSolver
{
public:
  struct Settings
  {
    int max_iterations{4000};
    double eps_abs{1.0e-4};
    double eps_rel{1.0e-4};
    double time_limit{0.0};  // [s], 0 = no limit
    bool polish{false};
    bool verbose{false};

    bool operator==(const Settings & other) const = default;
  };

  struct Result
  {
    bool solved{false};
    int iterations{0};
    std::string status;
    Eigen::VectorXd primal;
    double objective{0.0};
  };

  OsqpSolver();
  explicit OsqpSolver(const Settings & settings);
  ~OsqpSolver();
  OsqpSolver(const OsqpSolver &) = delete;
  OsqpSolver & operator=(const OsqpSolver &) = delete;
  OsqpSolver(OsqpSolver &&) noexcept;
  OsqpSolver & operator=(OsqpSolver &&) noexcept;

  // Changing the settings drops the workspace: the next solve performs a new setup.
  void setSettings(const Settings & settings);
  const Settings & settings() const;

  // Solves the QP; `warm_start` (primal) is optional. "solved inaccurate" counts as solved.
  Result solve(const QpData & qp, const Eigen::VectorXd * warm_start = nullptr);

  // Frees the workspace.
  void reset();

  // Number of full setups performed so far (useful to verify that updates are used).
  int setupCount() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mpc_core

#endif  // MPC_CORE__OSQP_SOLVER_HPP_
