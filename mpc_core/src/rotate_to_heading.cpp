#include "mpc_core/rotate_to_heading.hpp"

#include <algorithm>
#include <cmath>

namespace mpc_core
{

ControlVec rotateInPlaceCommand(
  double heading_error, const ControlVec & u_prev, const Limits & limits, double dt, double gain)
{
  const double w_stop = std::sqrt(2.0 * limits.alpha_max * std::abs(heading_error));
  const double w_cap = std::min(limits.w_max, w_stop);
  double w = std::clamp(gain * heading_error, -w_cap, w_cap);
  const double dw = limits.alpha_max * dt;
  w = std::clamp(w, u_prev.y() - dw, u_prev.y() + dw);

  const double dv = limits.a_max * dt;
  const double v =
    u_prev.x() > 0.0 ? std::max(0.0, u_prev.x() - dv) : std::min(0.0, u_prev.x() + dv);
  return {v, w};
}

}  // namespace mpc_core
