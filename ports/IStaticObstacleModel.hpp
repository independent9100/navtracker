#pragma once

#include <vector>

#include <Eigen/Core>

#include "core/types/StaticObstacle.hpp"

namespace navtracker {

/**
 * Charted static-hazard model, queried by the tracker at birth time and by
 * the hazard-output layer. Pure, zero-I/O. Nullable in use: if not wired,
 * behaviour is exactly today's. Separate from ILandModel (ADR 0002 decision
 * 4): discrete typed hazards, not a coastline suppression region.
 */
class IStaticObstacleModel {
 public:
  virtual ~IStaticObstacleModel() = default;

  /**
   * Vessel-birth suppression prior in [0,1] at ENU position (metres):
   *   0.0 = clear water (no suppression)
   *   1.0 = inside a charted obstacle's hard footprint (hard-gate region)
   * No nearby obstacle -> 0.0.
   */
  virtual double birthSuppression(const Eigen::Vector2d& enu_xy) const = 0;

  /** The active charted obstacles (for the hazard output + proximity alarm). */
  virtual const std::vector<StaticObstacle>& obstacles() const = 0;
};

}  // namespace navtracker
