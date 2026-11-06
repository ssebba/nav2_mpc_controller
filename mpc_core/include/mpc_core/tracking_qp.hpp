#ifndef MPC_CORE__TRACKING_QP_HPP_
#define MPC_CORE__TRACKING_QP_HPP_

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "mpc_core/types.hpp"

namespace mpc_core
{

// Admissible range of a command applied `period` seconds after `u_prev`: input box
// intersected with the rate limits. If the intersection is empty (e.g. right after a speed
// limit reduction) the box wins and the range collapses to the box value closest to `u_prev`.
void commandBounds(
  const MpcConfig & config, const ControlVec & u_prev, double period, ControlVec & lower,
  ControlVec & upper);

// Projects a control sequence onto the box and rate limits, step by step (the first step
// over the control period, the others over dt). Applied to the QP solution to remove
// constraint violations within the solver tolerance.
void projectControls(const MpcConfig & config, const ControlVec & u_prev, ControlTrajectory & u);

// Sparse QP in OSQP form: min 0.5 z^T P z + q^T z  s.t.  l <= A z <= u.
struct QpData
{
  Eigen::SparseMatrix<double> hessian;      // P (upper triangular part only)
  Eigen::VectorXd gradient;                 // q
  Eigen::SparseMatrix<double> constraints;  // A
  Eigen::VectorXd lower;                    // l (-inf allowed)
  Eigen::VectorXd upper;                    // u (+inf allowed)
};

/**
 * @brief Sparse ("multiple shooting") QP of the tracking MPC linearized around a nominal trajectory.
 *
 * Decision variables: z = [x_0 ... x_N, u_0 ... u_{N-1}, s_1 ... s_N], where s_k >= 0 are the
 * slacks that soften the obstacle constraints of step k.
 *
 * Cost (constant terms dropped):
 *   sum_{k=1}^{N} 1/2 |x_k - r_k|^2_{Q_k}        Q_N = terminal weight
 *
 * Q_k = diag(q_x, q_y, q_theta), except for the samples with a lateral weight scale
 * s_k < 1, whose position block is expressed along (t) and across (n) the reference heading:
 * q_t t t^T + s_k q_n n n^T (q_t = t^T diag(q_x, q_y) t, q_n likewise) and the heading weight
 * is s_k q_theta. The (x, y) off-diagonal entry is always stored, so the pattern is constant.
 * + sum_{k=0}^{N-1} 1/2 |u_k - u_ref_k|^2_R
 * + sum_{k=0}^{N-1} 1/2 |u_k - u_{k-1}|^2_{R_delta}   with u_{-1} = previous command
 * + sum_{k=1}^{N} (rho_1 s_k + 1/2 rho_2 s_k^2)
 *
 * Constraints (rows, in this order):
 *   x_0 = x_init
 *   A_k x_k + B_k u_k - x_{k+1} = -c_k               linearized dynamics
 *   u_min <= u_k <= u_max                            (u_0 also within rate limits of u_{-1})
 *   -du_max <= u_k - u_{k-1} <= du_max               k = 1 ... N-1
 *   n_kj^T p_k + s_k >= b_kj                         obstacle half-planes, M per step
 *   s_k >= 0
 *
 * The sparsity pattern depends only on (N, M): unused obstacle rows keep explicit zero
 * coefficients and infinite bounds, so the solver can update values without a new setup.
 */
class TrackingQp
{
public:
  TrackingQp(int horizon, int constraints_per_stage);

  int horizon() const { return horizon_; }
  int constraintsPerStage() const { return constraints_per_stage_; }
  int numVariables() const;
  int numConstraints() const { return num_rows_; }

  int stateIndex(int k) const;    // k = 0 ... N
  int controlIndex(int k) const;  // k = 0 ... N-1
  int slackIndex(int k) const;    // k = 1 ... N

  /**
   * @param x_nominal, u_nominal Linearization trajectory (N+1 states, N controls)
   * @param obstacles Half-planes per step (size N+1, or empty to disable them)
   */
  void build(
    const MpcConfig & config, const StateVec & x0, const ControlVec & u_prev,
    const Reference & reference, const StateTrajectory & x_nominal,
    const ControlTrajectory & u_nominal, const StageConstraints & obstacles, QpData & qp) const;

  // State weight Q_k of a reference sample with heading `reference_yaw` (see the class doc).
  static Eigen::Matrix3d stateWeight(
    const Eigen::Vector3d & q, double reference_yaw, double lateral_scale);

  // Primal vector for warm starting (slacks set to zero).
  Eigen::VectorXd pack(const StateTrajectory & x, const ControlTrajectory & u) const;
  void unpack(
    const Eigen::VectorXd & z, StateTrajectory & x, ControlTrajectory & u,
    Eigen::VectorXd & slack) const;

private:
  void buildCost(
    const MpcConfig & config, const ControlVec & u_prev, const Reference & reference,
    QpData & qp) const;
  void buildConstraints(
    const MpcConfig & config, const StateVec & x0, const ControlVec & u_prev,
    const StateTrajectory & x_nominal, const ControlTrajectory & u_nominal,
    const StageConstraints & obstacles, QpData & qp) const;

  int horizon_;
  int constraints_per_stage_;
  // First row of each constraint block.
  int row_dynamics_;
  int row_inputs_;
  int row_rates_;
  int row_obstacles_;
  int row_slacks_;
  int num_rows_;
};

}  // namespace mpc_core

#endif  // MPC_CORE__TRACKING_QP_HPP_
