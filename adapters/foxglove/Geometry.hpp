#pragma once
#include <string>
#include <vector>
#include <Eigen/Core>
#include "core/types/Ids.hpp"

namespace navtracker::foxglove {

/** A 3D point in the ENU frame used to build Foxglove scene geometry. */
struct Pt { double x, y, z; };            // ENU, z held at 0 for plan-view geometry
/** An RGBA color (components in [0,1]) for scene entities. */
struct Rgba { double r, g, b, a; };

/**
 * Polyline (closed loop) approximating the 1-σ·k ellipse of a symmetric 2x2
 * covariance, centered at `center_enu`. `n` points around the loop.
 */
std::vector<Pt> covarianceEllipse(const Eigen::Vector2d& center_enu,
                                  const Eigen::Matrix2d& cov,
                                  double k = 2.0, int n = 48);

/**
 * Bearing-only wedge: two rays from `sensor_enu` at angle alpha_rad ± k·sigma,
 * out to `length_m`, returned as a 3-point open polyline [edge1, apex, edge2].
 */
std::vector<Pt> bearingWedge(const Eigen::Vector2d& sensor_enu,
                             double alpha_rad, double sigma_rad,
                             double length_m, double k = 2.0);

/**
 * Closed circle (line loop) of `n` points, radius `radius_m`, centered at
 * `center_enu`. Used for obstacle footprint / keep-clear rings, vessel-fix
 * veto rings, and CPA threshold rings. The loop is closed (last point == first).
 */
std::vector<Pt> circle(const Eigen::Vector2d& center_enu, double radius_m,
                       int n = 48);

/**
 * Closed sector (pie wedge) outline: apex at `center_enu`, spanning
 * `center_rad ± half_width_rad` out to `range_m`, sampled with `n` arc points.
 * Returned as a closed polyline [apex, arc..., apex]. `center_rad` is the ENU
 * bearing (CCW from +x/east). Used for sensor coverage sectors; pass
 * half_width_rad >= pi for an omni disc (falls back to a full circle).
 */
std::vector<Pt> sectorArc(const Eigen::Vector2d& center_enu, double center_rad,
                          double half_width_rad, double range_m, int n = 32);

/** Stable per-sensor color (so the same source keeps its hue across frames). */
Rgba colorForSensor(SensorKind sensor, const std::string& source_id);

}  // namespace navtracker::foxglove
