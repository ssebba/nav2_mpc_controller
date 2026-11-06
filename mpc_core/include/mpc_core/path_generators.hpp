#ifndef MPC_CORE__PATH_GENERATORS_HPP_
#define MPC_CORE__PATH_GENERATORS_HPP_

#include "mpc_core/types.hpp"

namespace mpc_core::paths
{

// Reference geometries used by the tests and the tracking benchmark. All of them are
// sampled every ~`step` meters, start at `origin` and carry tangent yaw angles.

// Straight segment of length `length` along the x axis (shifted by `origin`).
StateTrajectory straight(double length, double step, const Point2d & origin = Point2d::Zero());

// Lemniscate of Gerono: x = a sin t, y = a sin t cos t. Total length ~6.1 a.
StateTrajectory figureEight(double size, double step, const Point2d & origin = Point2d::Zero());

// Sinusoid y = amplitude * sin(2 pi x / wavelength) over `length` meters of x.
StateTrajectory sineWave(
  double length, double amplitude, double wavelength, double step,
  const Point2d & origin = Point2d::Zero());

}  // namespace mpc_core::paths

#endif  // MPC_CORE__PATH_GENERATORS_HPP_
