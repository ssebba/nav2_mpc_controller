#include "mpc_core/linearized_mpc_solver.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>

#include "mpc_core/obstacle_constraints.hpp"
#include "mpc_core/unicycle_model.hpp"

namespace mpc_core
{

namespace
{

// Raises |v| of a linearization point to at least `min_speed`, in the direction of travel.
void applySpeedFloor(ControlTrajectory & u, double min_speed, double direction)
{
  for (auto & uk : u) {
    if (std::abs(uk.x()) < min_speed) {
      uk.x() = direction * min_speed;
    }
  }
}

}  // namespace

LinearizedMpcSolver::LinearizedMpcSolver(Linearization linearization)
: linearization_(linearization)
{
}

std::string LinearizedMpcSolver::name() const
{
  return linearization_ == Linearization::kReference ? "ltv_mpc" : "nmpc_sqp";
}

void LinearizedMpcSolver::configure(const MpcConfig & config)
{
  if (config.horizon < 2 || config.dt <= 0.0) {
    throw std::invalid_argument("LinearizedMpcSolver: horizon must be >= 2 and dt > 0");
  }
  if (config.obstacles.blocked_reference_weight < 0.0) {
    throw std::invalid_argument("LinearizedMpcSolver: blocked_reference_weight must be >= 0");
  }
  const int constraints_per_stage =
    config.obstacles.enabled ? std::max(0, config.obstacles.max_constraints_per_stage) : 0;
  if (
    !qp_ || qp_->horizon() != config.horizon ||
    qp_->constraintsPerStage() != constraints_per_stage) {
    qp_ = std::make_unique<TrackingQp>(config.horizon, constraints_per_stage);
    qp_solver_.reset();
    previous_controls_.clear();
  }
  OsqpSolver::Settings settings;
  settings.max_iterations = config.solver.max_qp_iterations;
  settings.eps_abs = config.solver.eps_abs;
  settings.eps_rel = config.solver.eps_rel;
  settings.time_limit = config.solver.max_solve_time;
  qp_solver_.setSettings(settings);

  config_ = config;
  initial_guess_.configure(config);
}

void LinearizedMpcSolver::reset()
{
  previous_controls_.clear();
  consecutive_failures_ = 0;
}

ControlTrajectory LinearizedMpcSolver::shiftedPreviousControls() const
{
  ControlTrajectory shifted(previous_controls_.begin() + 1, previous_controls_.end());
  shifted.push_back(previous_controls_.back());
  return shifted;
}

void LinearizedMpcSolver::nominalTrajectory(
  const StateVec & x0, const ControlVec & u_prev, const Reference & reference,
  StateTrajectory & x_nominal, ControlTrajectory & u_nominal)
{
  if (linearization_ == Linearization::kReference) {
    x_nominal = reference.states;
    u_nominal = reference.controls;
    return;
  }
  if (previous_controls_.size() == static_cast<std::size_t>(config_.horizon)) {
    u_nominal = shiftedPreviousControls();
  } else {
    u_nominal = initial_guess_.solve(x0, u_prev, reference, {}).controls;
  }
  applySpeedFloor(
    u_nominal, config_.solver.linearization_min_speed, config_.v_ref >= 0.0 ? 1.0 : -1.0);
  // Simulating the nonlinear model gives a dynamically consistent linearization point.
  x_nominal = UnicycleModel::rollout(x0, u_nominal, config_.dt);
}

MpcSolution LinearizedMpcSolver::solve(
  const StateVec & x0, const ControlVec & u_prev, const Reference & reference,
  const ObstaclePoints & obstacles)
{
  if (!qp_) {
    throw std::logic_error("LinearizedMpcSolver: solve() called before configure()");
  }
  const auto t_start = std::chrono::steady_clock::now();
  const auto n = static_cast<std::size_t>(config_.horizon);

  StateTrajectory x_nominal;
  ControlTrajectory u_nominal;
  nominalTrajectory(x0, u_prev, reference, x_nominal, u_nominal);

  // Where the plan crosses an obstacle the planner did not see, only progress along it is
  // tracked (see ObstacleConfig::blocked_reference_weight).
  Reference tracked = reference;
  if (config_.obstacles.enabled && !obstacles.empty()) {
    tracked.lateral_weight_scale =
      referenceWeightScale(reference.states, obstacles, config_.obstacles);
  }

  // Warm start from the shifted previous solution when available.
  Eigen::VectorXd warm_start;
  if (previous_controls_.size() == n) {
    const ControlTrajectory u_shift = shiftedPreviousControls();
    warm_start = qp_->pack(UnicycleModel::rollout(x0, u_shift, config_.dt), u_shift);
  } else {
    warm_start = qp_->pack(x_nominal, u_nominal);
  }

  const int max_iterations =
    linearization_ == Linearization::kReference ? 1 : std::max(1, config_.solver.sqp_iterations);
  MpcSolution sol;
  StateTrajectory x_qp;
  ControlTrajectory u_qp;
  Eigen::VectorXd slack;
  bool solved = false;
  for (int it = 0; it < max_iterations; ++it) {
    sol.constraints =
      buildObstacleConstraints(x_nominal, reference.states, obstacles, config_.obstacles);
    relaxCurrentViolations(
      sol.constraints, x0.head<2>(), config_.obstacles.violation_recovery_steps);
    qp_->build(config_, x0, u_prev, tracked, x_nominal, u_nominal, sol.constraints, qp_data_);
    const OsqpSolver::Result result = qp_solver_.solve(qp_data_, &warm_start);
    sol.qp_iterations += result.iterations;
    solved = result.solved;
    if (!solved) {
      break;
    }
    qp_->unpack(result.primal, x_qp, u_qp, slack);
    sol.sqp_iterations = it + 1;

    double step = 0.0;
    for (std::size_t k = 0; k < n; ++k) {
      step = std::max(step, (u_qp[k] - u_nominal[k]).cwiseAbs().maxCoeff());
    }
    if (step < config_.solver.sqp_tolerance) {
      break;
    }
    // Next SQP iteration: linearize around the new solution.
    x_nominal = x_qp;
    u_nominal = u_qp;
    applySpeedFloor(
      u_nominal, config_.solver.linearization_min_speed, config_.v_ref >= 0.0 ? 1.0 : -1.0);
    warm_start = result.primal;
  }

  if (solved) {
    consecutive_failures_ = 0;
    previous_controls_ = u_qp;
    sol.status = SolveStatus::kSuccess;
    sol.controls = u_qp;
    sol.max_slack = slack.size() > 0 ? std::max(0.0, slack.maxCoeff()) : 0.0;
  } else {
    ++consecutive_failures_;
    if (
      previous_controls_.size() == n &&
      consecutive_failures_ <= config_.solver.max_consecutive_failures) {
      previous_controls_ = shiftedPreviousControls();
      sol.status = SolveStatus::kFallback;
      sol.controls = previous_controls_;
    } else {
      previous_controls_.clear();
      sol.status = SolveStatus::kFailure;
    }
  }

  if (sol.ok()) {
    projectControls(config_, u_prev, sol.controls);
    previous_controls_ = sol.controls;
    sol.predicted_states = UnicycleModel::rollout(x0, sol.controls, config_.dt);
  }
  sol.solve_time_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_start).count();
  return sol;
}

}  // namespace mpc_core
