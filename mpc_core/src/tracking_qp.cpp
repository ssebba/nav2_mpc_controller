#include "mpc_core/tracking_qp.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "mpc_core/unicycle_model.hpp"

namespace mpc_core
{

namespace
{

using Triplet = Eigen::Triplet<double>;
constexpr double kInf = std::numeric_limits<double>::infinity();

void checkSize(std::size_t actual, int expected, const char * what)
{
  if (actual != static_cast<std::size_t>(expected)) {
    throw std::invalid_argument(
      std::string("TrackingQp: ") + what + " has size " + std::to_string(actual) + ", expected " +
      std::to_string(expected));
  }
}

}  // namespace

void commandBounds(
  const MpcConfig & config, const ControlVec & u_prev, double period, ControlVec & lower,
  ControlVec & upper)
{
  const auto & lim = config.limits;
  const ControlVec box_lo(lim.v_min, -lim.w_max);
  const ControlVec box_hi(lim.v_max, lim.w_max);
  const ControlVec du_max(lim.a_max * period, lim.alpha_max * period);
  for (int i = 0; i < kControlDim; ++i) {
    lower(i) = std::max(box_lo(i), u_prev(i) - du_max(i));
    upper(i) = std::min(box_hi(i), u_prev(i) + du_max(i));
    if (lower(i) > upper(i)) {
      lower(i) = upper(i) = std::clamp(u_prev(i), box_lo(i), box_hi(i));
    }
  }
}

void projectControls(const MpcConfig & config, const ControlVec & u_prev, ControlTrajectory & u)
{
  ControlVec last = u_prev;
  double period = config.firstStepPeriod();
  for (auto & uk : u) {
    ControlVec lo;
    ControlVec hi;
    commandBounds(config, last, period, lo, hi);
    period = config.dt;
    uk = uk.cwiseMax(lo).cwiseMin(hi);
    last = uk;
  }
}

Eigen::Matrix3d TrackingQp::stateWeight(
  const Eigen::Vector3d & q, double reference_yaw, double lateral_scale)
{
  Eigen::Matrix3d w = q.asDiagonal();
  if (lateral_scale >= 1.0) {
    return w;
  }
  const Eigen::Vector2d t(std::cos(reference_yaw), std::sin(reference_yaw));
  const Eigen::Vector2d n(-t.y(), t.x());
  const Eigen::Matrix2d d = q.head<2>().asDiagonal();
  w.topLeftCorner<2, 2>() =
    t.dot(d * t) * t * t.transpose() + lateral_scale * n.dot(d * n) * n * n.transpose();
  w(2, 2) = lateral_scale * q(2);
  return w;
}

TrackingQp::TrackingQp(int horizon, int constraints_per_stage)
: horizon_(horizon), constraints_per_stage_(constraints_per_stage)
{
  if (horizon_ < 1 || constraints_per_stage_ < 0) {
    throw std::invalid_argument("TrackingQp: invalid dimensions");
  }
  row_dynamics_ = kStateDim;  // after the initial state rows
  row_inputs_ = row_dynamics_ + kStateDim * horizon_;
  row_rates_ = row_inputs_ + kControlDim * horizon_;
  row_obstacles_ = row_rates_ + kControlDim * (horizon_ - 1);
  row_slacks_ = row_obstacles_ + constraints_per_stage_ * horizon_;
  num_rows_ = row_slacks_ + horizon_;
}

int TrackingQp::numVariables() const
{
  return kStateDim * (horizon_ + 1) + kControlDim * horizon_ + horizon_;
}

int TrackingQp::stateIndex(int k) const { return kStateDim * k; }

int TrackingQp::controlIndex(int k) const { return kStateDim * (horizon_ + 1) + kControlDim * k; }

int TrackingQp::slackIndex(int k) const
{
  return kStateDim * (horizon_ + 1) + kControlDim * horizon_ + (k - 1);
}

void TrackingQp::build(
  const MpcConfig & config, const StateVec & x0, const ControlVec & u_prev,
  const Reference & reference, const StateTrajectory & x_nominal,
  const ControlTrajectory & u_nominal, const StageConstraints & obstacles, QpData & qp) const
{
  checkSize(reference.states.size(), horizon_ + 1, "reference.states");
  checkSize(reference.controls.size(), horizon_, "reference.controls");
  checkSize(x_nominal.size(), horizon_ + 1, "x_nominal");
  checkSize(u_nominal.size(), horizon_, "u_nominal");
  if (!obstacles.empty()) {
    checkSize(obstacles.size(), horizon_ + 1, "obstacles");
  }
  if (!reference.lateral_weight_scale.empty()) {
    checkSize(
      reference.lateral_weight_scale.size(), horizon_ + 1, "reference.lateral_weight_scale");
  }
  buildCost(config, u_prev, reference, qp);
  buildConstraints(config, x0, u_prev, x_nominal, u_nominal, obstacles, qp);
}

void TrackingQp::buildCost(
  const MpcConfig & config, const ControlVec & u_prev, const Reference & reference,
  QpData & qp) const
{
  const auto & w = config.weights;
  const auto & obs = config.obstacles;
  const int nv = numVariables();

  std::vector<Triplet> p;
  const auto n = static_cast<std::size_t>(horizon_);
  const auto nx = static_cast<std::size_t>(kStateDim);
  const auto nu = static_cast<std::size_t>(kControlDim);
  p.reserve((nx * n) + (2 * nu * n) + n);
  qp.gradient.setZero(nv);

  for (int k = 1; k <= horizon_; ++k) {
    const auto ks = static_cast<std::size_t>(k);
    const Eigen::Vector3d & qk = (k == horizon_) ? w.q_terminal : w.q;
    const double scale =
      reference.lateral_weight_scale.empty() ? 1.0 : reference.lateral_weight_scale[ks];
    const Eigen::Matrix3d q = stateWeight(qk, reference.states[ks].z(), scale);
    const int x = stateIndex(k);
    // Upper triangular part; the (x, y) entry is written even when zero, so that the
    // sparsity pattern does not depend on the weight scale.
    p.emplace_back(x, x, q(0, 0));
    p.emplace_back(x, x + 1, q(0, 1));
    p.emplace_back(x + 1, x + 1, q(1, 1));
    p.emplace_back(x + 2, x + 2, q(2, 2));
    qp.gradient.segment<kStateDim>(x) = -q * reference.states[ks];
  }

  for (int k = 0; k < horizon_; ++k) {
    for (int i = 0; i < kControlDim; ++i) {
      const int idx = controlIndex(k) + i;
      // u_k appears in the increments (u_k - u_{k-1}) and, if not last, (u_{k+1} - u_k).
      const double r_delta_terms = (k + 1 < horizon_) ? 2.0 * w.r_delta(i) : w.r_delta(i);
      p.emplace_back(idx, idx, w.r(i) + r_delta_terms);
      if (k > 0) {
        // Upper triangular coupling between u_{k-1} and u_k.
        p.emplace_back(controlIndex(k - 1) + i, idx, -w.r_delta(i));
      }
      qp.gradient(idx) = -w.r(i) * reference.controls[k](i);
    }
  }
  for (int i = 0; i < kControlDim; ++i) {
    qp.gradient(controlIndex(0) + i) -= w.r_delta(i) * u_prev(i);
  }

  for (int k = 1; k <= horizon_; ++k) {
    p.emplace_back(slackIndex(k), slackIndex(k), obs.slack_quadratic_weight);
    qp.gradient(slackIndex(k)) = obs.slack_linear_weight;
  }

  qp.hessian.resize(nv, nv);
  qp.hessian.setFromTriplets(p.begin(), p.end());
  qp.hessian.makeCompressed();
}

void TrackingQp::buildConstraints(
  const MpcConfig & config, const StateVec & x0, const ControlVec & u_prev,
  const StateTrajectory & x_nominal, const ControlTrajectory & u_nominal,
  const StageConstraints & obstacles, QpData & qp) const
{
  const auto & lim = config.limits;
  const double dt = config.dt;
  std::vector<Triplet> a;
  const auto n = static_cast<std::size_t>(horizon_);
  const auto m = static_cast<std::size_t>(constraints_per_stage_);
  const auto nx = static_cast<std::size_t>(kStateDim);
  const auto nu = static_cast<std::size_t>(kControlDim);
  // Initial state, dynamics (11 entries per step), input box, rates, obstacles, slacks.
  a.reserve(nx + (11 * n) + (nu * n) + (2 * nu * n) + (3 * m * n) + n);
  qp.lower.resize(num_rows_);
  qp.upper.resize(num_rows_);

  // Initial state.
  for (int i = 0; i < kStateDim; ++i) {
    a.emplace_back(i, stateIndex(0) + i, 1.0);
    qp.lower(i) = qp.upper(i) = x0(i);
  }

  // Linearized dynamics. The pattern of A_k and B_k is fixed by the unicycle structure
  // (entries are written even when they evaluate to zero, e.g. for v = 0).
  for (int k = 0; k < horizon_; ++k) {
    const auto ak = UnicycleModel::jacobianState(x_nominal[k], u_nominal[k], dt);
    const auto bk = UnicycleModel::jacobianControl(x_nominal[k], dt);
    const StateVec ck =
      UnicycleModel::step(x_nominal[k], u_nominal[k], dt) - ak * x_nominal[k] - bk * u_nominal[k];
    const int r = row_dynamics_ + kStateDim * k;
    const int xk = stateIndex(k);
    const int uk = controlIndex(k);
    a.emplace_back(r + 0, xk + 0, ak(0, 0));
    a.emplace_back(r + 1, xk + 1, ak(1, 1));
    a.emplace_back(r + 2, xk + 2, ak(2, 2));
    a.emplace_back(r + 0, xk + 2, ak(0, 2));
    a.emplace_back(r + 1, xk + 2, ak(1, 2));
    a.emplace_back(r + 0, uk + 0, bk(0, 0));
    a.emplace_back(r + 1, uk + 0, bk(1, 0));
    a.emplace_back(r + 2, uk + 1, bk(2, 1));
    for (int i = 0; i < kStateDim; ++i) {
      a.emplace_back(r + i, stateIndex(k + 1) + i, -1.0);
    }
    qp.lower.segment<kStateDim>(r) = -ck;
    qp.upper.segment<kStateDim>(r) = -ck;
  }

  // Input box, with the rate limit w.r.t. the previous command folded into u_0.
  const ControlVec box_lo(lim.v_min, -lim.w_max);
  const ControlVec box_hi(lim.v_max, lim.w_max);
  const ControlVec du_max(lim.a_max * dt, lim.alpha_max * dt);
  ControlVec first_lo;
  ControlVec first_hi;
  commandBounds(config, u_prev, config.firstStepPeriod(), first_lo, first_hi);
  for (int k = 0; k < horizon_; ++k) {
    const int r = row_inputs_ + kControlDim * k;
    for (int i = 0; i < kControlDim; ++i) {
      a.emplace_back(r + i, controlIndex(k) + i, 1.0);
      qp.lower(r + i) = k == 0 ? first_lo(i) : box_lo(i);
      qp.upper(r + i) = k == 0 ? first_hi(i) : box_hi(i);
    }
  }

  // Input rates.
  for (int k = 1; k < horizon_; ++k) {
    const int r = row_rates_ + kControlDim * (k - 1);
    for (int i = 0; i < kControlDim; ++i) {
      a.emplace_back(r + i, controlIndex(k - 1) + i, -1.0);
      a.emplace_back(r + i, controlIndex(k) + i, 1.0);
      qp.lower(r + i) = -du_max(i);
      qp.upper(r + i) = du_max(i);
    }
  }

  // Obstacle half-planes (soft).
  for (int k = 1; k <= horizon_; ++k) {
    for (int j = 0; j < constraints_per_stage_; ++j) {
      const int r = row_obstacles_ + constraints_per_stage_ * (k - 1) + j;
      const bool active = !obstacles.empty() && static_cast<std::size_t>(j) <
                                                  obstacles[static_cast<std::size_t>(k)].size();
      const HalfPlane hp = active
                             ? obstacles[static_cast<std::size_t>(k)][static_cast<std::size_t>(j)]
                             : HalfPlane{Point2d::Zero(), 0.0, Point2d::Zero()};
      a.emplace_back(r, stateIndex(k) + 0, hp.normal.x());
      a.emplace_back(r, stateIndex(k) + 1, hp.normal.y());
      a.emplace_back(r, slackIndex(k), 1.0);
      qp.lower(r) = active ? hp.offset : -kInf;
      qp.upper(r) = kInf;
    }
  }

  // Slacks are non-negative.
  for (int k = 1; k <= horizon_; ++k) {
    const int r = row_slacks_ + (k - 1);
    a.emplace_back(r, slackIndex(k), 1.0);
    qp.lower(r) = 0.0;
    qp.upper(r) = kInf;
  }

  qp.constraints.resize(num_rows_, numVariables());
  qp.constraints.setFromTriplets(a.begin(), a.end());
  qp.constraints.makeCompressed();
}

Eigen::VectorXd TrackingQp::pack(const StateTrajectory & x, const ControlTrajectory & u) const
{
  checkSize(x.size(), horizon_ + 1, "x");
  checkSize(u.size(), horizon_, "u");
  Eigen::VectorXd z = Eigen::VectorXd::Zero(numVariables());
  for (int k = 0; k <= horizon_; ++k) {
    z.segment<kStateDim>(stateIndex(k)) = x[static_cast<std::size_t>(k)];
  }
  for (int k = 0; k < horizon_; ++k) {
    z.segment<kControlDim>(controlIndex(k)) = u[static_cast<std::size_t>(k)];
  }
  return z;
}

void TrackingQp::unpack(
  const Eigen::VectorXd & z, StateTrajectory & x, ControlTrajectory & u,
  Eigen::VectorXd & slack) const
{
  checkSize(static_cast<std::size_t>(z.size()), numVariables(), "z");
  x.resize(static_cast<std::size_t>(horizon_) + 1);
  u.resize(static_cast<std::size_t>(horizon_));
  for (int k = 0; k <= horizon_; ++k) {
    x[static_cast<std::size_t>(k)] = z.segment<kStateDim>(stateIndex(k));
  }
  for (int k = 0; k < horizon_; ++k) {
    u[static_cast<std::size_t>(k)] = z.segment<kControlDim>(controlIndex(k));
  }
  slack = z.segment(slackIndex(1), horizon_);
}

}  // namespace mpc_core
