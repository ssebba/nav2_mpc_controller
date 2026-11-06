#ifndef MPC_CORE__TEST__TEST_UTILS_HPP_
#define MPC_CORE__TEST__TEST_UTILS_HPP_

#include "mpc_core/closed_loop.hpp"
#include "mpc_core/path_generators.hpp"
#include "mpc_core/types.hpp"

namespace mpc_core::test
{

inline StateTrajectory straightPath(double length, double step, double y = 0.0)
{
  return paths::straight(length, step, Point2d(0.0, y));
}

inline StateTrajectory figureEight(double size, double step)
{
  return paths::figureEight(size, step);
}

using mpc_core::runClosedLoop;

}  // namespace mpc_core::test

#endif  // MPC_CORE__TEST__TEST_UTILS_HPP_
