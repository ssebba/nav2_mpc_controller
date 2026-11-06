#ifndef MPC_CORE__TYPES_HPP_
#define MPC_CORE__TYPES_HPP_

#include <vector>

#include <Eigen/Core>

namespace mpc_core
{

constexpr int kStateDim = 3;    // [x, y, yaw]
constexpr int kControlDim = 2;  // [v, w]

using StateVec = Eigen::Vector3d;
using ControlVec = Eigen::Vector2d;
using Point2d = Eigen::Vector2d;

using StateTrajectory = std::vector<StateVec>;
using ControlTrajectory = std::vector<ControlVec>;
using ObstaclePoints = std::vector<Point2d>;

struct Weights
{
  // Stage cost on the tracking error [x, y, yaw].
  Eigen::Vector3d q{10.0, 10.0, 1.0};
  // Terminal cost on the tracking error [x, y, yaw].
  Eigen::Vector3d q_terminal{20.0, 20.0, 2.0};
  // Cost on the deviation from the feed-forward command [v, w].
  Eigen::Vector2d r{0.5, 0.1};
  // Cost on the command increments [dv, dw]: penalizes jerky commands.
  Eigen::Vector2d r_delta{5.0, 1.0};
};

struct Limits
{
  double v_min{-0.05};    // [m/s]
  double v_max{0.26};     // [m/s]
  double w_max{1.8};      // [rad/s], symmetric
  double a_max{1.0};      // [m/s^2], symmetric
  double alpha_max{3.0};  // [rad/s^2], symmetric
};

struct ObstacleConfig
{
  bool enabled{true};
  // Minimum distance between the robot center and any obstacle point [m].
  double safety_distance{0.3};
  // Obstacle points farther than this from a predicted position are ignored [m].
  double activation_distance{1.0};
  // Maximum number of half-plane constraints per prediction step.
  int max_constraints_per_stage{2};
  // Minimum angle between the normals of two constraints of the same step [rad].
  double min_angle_separation{0.78};
  // Maximum angle between a constraint normal and the lateral direction of travel [rad].
  // Normals of obstacles straight ahead are tilted sideways so the half-plane lets the
  // robot slide around the obstacle instead of stopping in front of it. pi/2 = radial.
  double escape_angle{0.78};
  // A constraint already violated at the current position (e.g. the robot starts inside the
  // safety margin) is relaxed by that violation, linearly decreasing to zero over this many
  // steps: the robot must move out of the margin, not jump out of it at the first step.
  int violation_recovery_steps{5};
  // Scale of the cross-track and heading weights for reference samples closer than
  // safety_distance to an obstacle point. There the plan crosses an obstacle the planner did
  // not know about: tracking it laterally only pulls the robot against the constraints, and
  // beside the obstacle the optimum can become "advance by almost nothing" at every cycle.
  // The along-track weight is kept, so the robot is still pulled forward along the path.
  double blocked_reference_weight{0.1};
  // Exact (L1) and quadratic penalties on the constraint violation.
  double slack_linear_weight{1.0e3};
  double slack_quadratic_weight{1.0e4};
};

struct SolverConfig
{
  // SQP iterations per control cycle (1 = Real-Time Iteration). Ignored by the LTV solver.
  int sqp_iterations{1};
  // SQP stops early when the max-norm of the step is below this value.
  double sqp_tolerance{1.0e-3};
  // Minimum |v| of the SQP linearization point [m/s]. At v = 0 the unicycle Jacobian loses
  // the yaw -> position coupling and the QP cannot "see" that turning lets it move around
  // an obstacle. Ignored by the LTV solver (linearized on the reference).
  double linearization_min_speed{0.1};
  // OSQP settings.
  int max_qp_iterations{4000};
  double eps_abs{1.0e-4};
  double eps_rel{1.0e-4};
  // Wall-clock budget of one QP solve [s]; exceeding it counts as a failure (fallback).
  double max_solve_time{0.025};
  // Number of consecutive failures tolerated by replaying the previous (shifted) solution.
  int max_consecutive_failures{3};
};

struct MpcConfig
{
  int horizon{20};  // Number of prediction steps N
  double dt{0.1};   // Discretization step [s]
  // Period of the control loop [s] (0 = dt). The first command is applied for this long, so
  // its rate limits w.r.t. the previous command are computed over this period.
  double control_period{0.0};
  double v_ref{0.25};  // Desired cruise speed used to sample the reference [m/s]
  Weights weights;
  Limits limits;
  ObstacleConfig obstacles;
  SolverConfig solver;

  double firstStepPeriod() const { return control_period > 0.0 ? control_period : dt; }
};

// Time-parametrized reference over the horizon.
struct Reference
{
  StateTrajectory states;      // r_0 ... r_N
  ControlTrajectory controls;  // feed-forward u_ref_0 ... u_ref_{N-1}
  // Optional scale of the cross-track and heading weights of each sample r_0 ... r_N, the
  // along-track weight being unchanged (empty: all 1, i.e. the plain weights Q).
  std::vector<double> lateral_weight_scale;
};

// Linear constraint on the robot position p at one prediction step: normal^T p >= offset.
struct HalfPlane
{
  Point2d normal{Point2d::UnitX()};
  double offset{0.0};
  // Obstacle point that generated the constraint (used for visualization).
  Point2d obstacle{Point2d::Zero()};
};

// Half-planes of every prediction step, indexed by k = 0 ... N (step 0 is always empty).
using StageConstraints = std::vector<std::vector<HalfPlane>>;

enum class SolveStatus
{
  kSuccess,   // Optimal solution found
  kFallback,  // Solver failed, the previous solution was shifted and replayed
  kFailure,   // No usable command
};

struct MpcSolution
{
  SolveStatus status{SolveStatus::kFailure};
  // Control sequence u_0 ... u_{N-1}. Only u_0 is applied (receding horizon).
  ControlTrajectory controls;
  // States x_0 ... x_N obtained by simulating the nonlinear model with `controls`.
  StateTrajectory predicted_states;
  // Obstacle constraints used in the last QP (for visualization).
  StageConstraints constraints;
  int qp_iterations{0};
  int sqp_iterations{0};
  // Largest obstacle constraint violation allowed by the slack variables [m].
  double max_slack{0.0};
  double solve_time_ms{0.0};

  bool ok() const { return status != SolveStatus::kFailure; }
};

}  // namespace mpc_core

#endif  // MPC_CORE__TYPES_HPP_
