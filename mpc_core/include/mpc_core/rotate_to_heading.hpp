#ifndef MPC_CORE__ROTATE_TO_HEADING_HPP_
#define MPC_CORE__ROTATE_TO_HEADING_HPP_

#include "mpc_core/types.hpp"

namespace mpc_core
{

/**
 * @brief In-place rotation command that reduces `heading_error` (target - current yaw).
 *
 * The angular velocity is proportional to the error, capped by w_max and by the
 * largest value that still allows stopping on the target with alpha_max
 * (|w| <= sqrt(2 alpha_max |error|)), then rate limited w.r.t. the previous command.
 * The linear velocity is braked towards zero within a_max.
 */
ControlVec rotateInPlaceCommand(
  double heading_error, const ControlVec & u_prev, const Limits & limits, double dt,
  double gain = 2.0);

}  // namespace mpc_core

#endif  // MPC_CORE__ROTATE_TO_HEADING_HPP_
