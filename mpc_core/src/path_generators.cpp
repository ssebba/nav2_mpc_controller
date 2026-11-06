#include "mpc_core/path_generators.hpp"

#include <cmath>
#include <functional>
#include <numbers>
#include <stdexcept>

#include "mpc_core/reference_builder.hpp"

namespace mpc_core::paths
{

namespace
{

// Samples a parametric curve c(t), t in [0, 1], with an approximately uniform arc-length step.
StateTrajectory sampleCurve(
  const std::function<Point2d(double)> & curve, double step, const Point2d & origin)
{
  if (step <= 0.0) {
    throw std::invalid_argument("paths: step must be positive");
  }
  // Dense pre-sampling to measure the length, then resampling.
  constexpr int kDense = 20000;
  StateTrajectory path;
  path.emplace_back(origin.x() + curve(0.0).x(), origin.y() + curve(0.0).y(), 0.0);
  double since_last = 0.0;
  Point2d prev = curve(0.0);
  for (int i = 1; i <= kDense; ++i) {
    const Point2d p = curve(static_cast<double>(i) / kDense);
    since_last += (p - prev).norm();
    prev = p;
    if (since_last >= step || i == kDense) {
      path.emplace_back(origin.x() + p.x(), origin.y() + p.y(), 0.0);
      since_last = 0.0;
    }
  }
  ReferenceBuilder::assignTangentYaw(path);
  // The last pose keeps the direction of the last segment.
  if (path.size() >= 2) {
    path.back().z() = path[path.size() - 2].z();
  }
  return path;
}

}  // namespace

StateTrajectory straight(double length, double step, const Point2d & origin)
{
  return sampleCurve([length](double t) { return Point2d(length * t, 0.0); }, step, origin);
}

StateTrajectory figureEight(double size, double step, const Point2d & origin)
{
  return sampleCurve(
    [size](double t) {
      const double a = 2.0 * std::numbers::pi * t;
      return Point2d(size * std::sin(a), size * std::sin(a) * std::cos(a));
    },
    step, origin);
}

StateTrajectory sineWave(
  double length, double amplitude, double wavelength, double step, const Point2d & origin)
{
  return sampleCurve(
    [=](double t) {
      const double x = length * t;
      return Point2d(x, amplitude * std::sin(2.0 * std::numbers::pi * x / wavelength));
    },
    step, origin);
}

}  // namespace mpc_core::paths
