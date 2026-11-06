#include "mpc_core/osqp_solver.hpp"

#include <osqp.h>

#include <algorithm>
#include <vector>

namespace mpc_core
{

namespace
{

// CSC copy of an Eigen sparse matrix with OSQP index/value types.
struct CscBuffer
{
  OSQPInt rows{0};
  OSQPInt cols{0};
  std::vector<OSQPInt> outer;
  std::vector<OSQPInt> inner;
  std::vector<OSQPFloat> values;

  void assign(const Eigen::SparseMatrix<double> & m)
  {
    rows = m.rows();
    cols = m.cols();
    outer.assign(m.outerIndexPtr(), m.outerIndexPtr() + m.outerSize() + 1);
    inner.assign(m.innerIndexPtr(), m.innerIndexPtr() + m.nonZeros());
    values.assign(m.valuePtr(), m.valuePtr() + m.nonZeros());
  }

  bool samePattern(const Eigen::SparseMatrix<double> & m) const
  {
    return rows == m.rows() && cols == m.cols() &&
           static_cast<Eigen::Index>(inner.size()) == m.nonZeros() &&
           std::equal(outer.begin(), outer.end(), m.outerIndexPtr()) &&
           std::equal(inner.begin(), inner.end(), m.innerIndexPtr());
  }

  OSQPCscMatrix view()
  {
    OSQPCscMatrix csc{};
    OSQPCscMatrix_set_data(
      &csc, rows, cols, static_cast<OSQPInt>(values.size()), values.data(), inner.data(),
      outer.data());
    return csc;
  }
};

std::vector<OSQPFloat> clampToOsqpInfinity(const Eigen::VectorXd & v)
{
  std::vector<OSQPFloat> out(static_cast<std::size_t>(v.size()));
  for (Eigen::Index i = 0; i < v.size(); ++i) {
    out[static_cast<std::size_t>(i)] = std::clamp<OSQPFloat>(v(i), -OSQP_INFTY, OSQP_INFTY);
  }
  return out;
}

}  // namespace

struct OsqpSolver::Impl
{
  Settings settings;
  OSQPSolver * solver{nullptr};
  CscBuffer p;
  CscBuffer a;
  int setup_count{0};

  ~Impl() { cleanup(); }

  void cleanup()
  {
    if (solver != nullptr) {
      osqp_cleanup(solver);
      solver = nullptr;
    }
  }

  OSQPSettings osqpSettings() const
  {
    OSQPSettings s{};
    osqp_set_default_settings(&s);
    s.max_iter = settings.max_iterations;
    s.eps_abs = settings.eps_abs;
    s.eps_rel = settings.eps_rel;
    s.polishing = settings.polish ? 1 : 0;
    s.verbose = settings.verbose ? 1 : 0;
    s.warm_starting = 1;
    // OSQP 1.0 also waits for a small duality gap before terminating. With the exact (L1)
    // slack penalty this criterion converges much more slowly than the residuals: a cold start
    // on the boundary of a safety margin needed more than 4000 iterations. Terminate on the
    // primal and dual residuals only, as OSQP 0.x does.
    s.check_dualgap = 0;
    if (settings.time_limit > 0.0) {
      s.time_limit = settings.time_limit;
    }
    return s;
  }
};

OsqpSolver::OsqpSolver() : OsqpSolver(Settings{}) {}

OsqpSolver::OsqpSolver(const Settings & settings) : impl_(std::make_unique<Impl>())
{
  impl_->settings = settings;
}

OsqpSolver::~OsqpSolver() = default;
OsqpSolver::OsqpSolver(OsqpSolver &&) noexcept = default;
OsqpSolver & OsqpSolver::operator=(OsqpSolver &&) noexcept = default;

void OsqpSolver::setSettings(const Settings & settings)
{
  if (!(settings == impl_->settings)) {
    impl_->settings = settings;
    impl_->cleanup();
  }
}

const OsqpSolver::Settings & OsqpSolver::settings() const { return impl_->settings; }

void OsqpSolver::reset() { impl_->cleanup(); }

int OsqpSolver::setupCount() const { return impl_->setup_count; }

OsqpSolver::Result OsqpSolver::solve(const QpData & qp, const Eigen::VectorXd * warm_start)
{
  Impl & d = *impl_;
  Result result;

  const std::vector<OSQPFloat> q(qp.gradient.data(), qp.gradient.data() + qp.gradient.size());
  const std::vector<OSQPFloat> l = clampToOsqpInfinity(qp.lower);
  const std::vector<OSQPFloat> u = clampToOsqpInfinity(qp.upper);

  const bool reuse =
    d.solver != nullptr && d.p.samePattern(qp.hessian) && d.a.samePattern(qp.constraints);
  if (reuse) {
    d.p.assign(qp.hessian);
    d.a.assign(qp.constraints);
    // Passing NULL index arrays replaces all the values (same pattern guaranteed above).
    OSQPInt err = osqp_update_data_mat(
      d.solver, d.p.values.data(), nullptr, static_cast<OSQPInt>(d.p.values.size()),
      d.a.values.data(), nullptr, static_cast<OSQPInt>(d.a.values.size()));
    if (err == 0) {
      err = osqp_update_data_vec(d.solver, q.data(), l.data(), u.data());
    }
    if (err != 0) {
      result.status = osqp_error_message(err);
      d.cleanup();
      return result;
    }
  } else {
    d.cleanup();
    d.p.assign(qp.hessian);
    d.a.assign(qp.constraints);
    OSQPCscMatrix p_csc = d.p.view();
    OSQPCscMatrix a_csc = d.a.view();
    const OSQPSettings settings = d.osqpSettings();
    const OSQPInt err = osqp_setup(
      &d.solver, &p_csc, q.data(), &a_csc, l.data(), u.data(), d.a.rows, d.a.cols, &settings);
    if (err != 0) {
      result.status = osqp_error_message(err);
      d.cleanup();
      return result;
    }
    ++d.setup_count;
  }

  if (warm_start != nullptr && warm_start->size() == qp.gradient.size()) {
    osqp_warm_start(d.solver, warm_start->data(), nullptr);
  }

  osqp_solve(d.solver);
  const OSQPInfo & info = *d.solver->info;
  result.status = info.status;
  result.iterations = static_cast<int>(info.iter);
  result.objective = info.obj_val;
  result.solved = info.status_val == OSQP_SOLVED || info.status_val == OSQP_SOLVED_INACCURATE;
  if (result.solved) {
    result.primal = Eigen::Map<const Eigen::VectorXd>(d.solver->solution->x, qp.gradient.size());
  }
  return result;
}

}  // namespace mpc_core
