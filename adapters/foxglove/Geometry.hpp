#pragma once
#include <array>
#include <vector>
#include <Eigen/Core>
#include "core/types/Ids.hpp"

namespace navtracker::foxglove {

struct Pt { double x, y, z; };            // ENU, z held at 0 for plan-view geometry
struct Rgba { double r, g, b, a; };

// Polyline (closed loop) approximating the 1-σ·k ellipse of a symmetric 2x2
// covariance, centered at `center_enu`. `n` points around the loop.
std::vector<Pt> covarianceEllipse(const Eigen::Vector2d& center_enu,
                                  const Eigen::Matrix2d& cov,
                                  double k = 2.0, int n = 48);

// Bearing-only wedge: two rays from `sensor_enu` at angle alpha_rad ± k·sigma,
// out to `length_m`, returned as a 3-point open polyline [edge1, apex, edge2].
std::vector<Pt> bearingWedge(const Eigen::Vector2d& sensor_enu,
                             double alpha_rad, double sigma_rad,
                             double length_m, double k = 2.0);

// Stable per-sensor color (so the same source keeps its hue across frames).
Rgba colorForSensor(SensorKind sensor, const std::string& source_id);

}  // namespace navtracker::foxglove
