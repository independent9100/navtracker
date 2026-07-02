#pragma once

#include <algorithm>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "core/geo/Datum.hpp"
#include "core/own_ship/OwnShipProvider.hpp"  // IDatumChangeSink
#include "core/types/StaticObstacle.hpp"
#include "ports/IStaticObstacleModel.hpp"

namespace navtracker {

struct StaticObstacleParams {
  // Max soft suppression at the outer edge of the footprint. MUST be strictly
  // below the tracker's static_obstacle_hard_gate (default 0.95) so the
  // keep-clear buffer is soft-only: a real vessel passing close still births.
  // Only the hard footprint interior (c=1.0) triggers the hard gate.
  double soft_max = 0.9;
};

// Concrete static-obstacle model: holds charted obstacles (geodetic) + the
// working datum, precomputing their ENU positions for fast proximity queries.
// birthSuppression is a soft ramp c(d) over distance d to the nearest obstacle:
//   d <= R_hard            -> 1.0                              (footprint core)
//   R_hard < d <= R_soft   -> soft_max*(R_soft-d)/(R_soft-R_hard)  (buffer ramp)
//   d > R_soft             -> 0.0                              (clear water)
// with R_hard = footprint_radius_m + position_uncertainty_m and
//      R_soft  = max(keep_clear_radius_m, R_hard).
// Pure: no I/O, no wall-clock, no RNG. Mirrors CoastlineModel. On datum
// recenter the ENU cache is rebuilt (deterministic).
class StaticObstacleModel : public IStaticObstacleModel, public IDatumChangeSink {
 public:
  StaticObstacleModel(std::vector<StaticObstacle> obstacles, geo::Datum datum,
                      StaticObstacleParams params = {})
      : obstacles_(std::move(obstacles)), datum_(datum), params_(params) {
    // R7.1: cap soft_max at construction so birthSuppression (which approaches
    // soft_max at the footprint edge) stays strictly below the tracker's DEFAULT
    // static_obstacle_hard_gate (0.95), keeping the keep-clear buffer soft-only
    // (only the c=1.0 footprint interior hard-drops). Finding #7: this pure model
    // cannot see the tracker's Config, so the 0.9 cap only guarantees the
    // invariant for the default gate. A caller that lowers static_obstacle_hard_gate
    // below 0.9 MUST lower soft_max below its chosen gate too — enforcing
    // soft_max < static_obstacle_hard_gate is a composition-root responsibility
    // (both values are visible only where the model meets the Config).
    params_.soft_max = std::min(params_.soft_max, 0.9);
    rebuildEnu();
  }

  double birthSuppression(const Eigen::Vector2d& enu_xy) const override {
    double c = 0.0;
    for (std::size_t i = 0; i < enu_.size(); ++i) {
      const double d = (enu_[i] - enu_xy).norm();
      const double r_hard =
          obstacles_[i].footprint_radius_m + obstacles_[i].position_uncertainty_m;
      const double r_soft = std::max(obstacles_[i].keep_clear_radius_m, r_hard);
      double ci;
      if (d <= r_hard) {
        ci = 1.0;
      } else if (d <= r_soft && r_soft > r_hard) {
        ci = params_.soft_max * (r_soft - d) / (r_soft - r_hard);
      } else {
        ci = 0.0;
      }
      c = std::max(c, ci);
    }
    return c;
  }

  const std::vector<StaticObstacle>& obstacles() const override {
    return obstacles_;
  }

  void onDatumRecentered(const geo::Datum& /*old_datum*/,
                         const geo::Datum& new_datum) override {
    datum_ = new_datum;
    rebuildEnu();
  }

 private:
  void rebuildEnu() {
    enu_.clear();
    enu_.reserve(obstacles_.size());
    for (const auto& o : obstacles_) {
      const Eigen::Vector3d e = datum_.toEnu(o.position);
      enu_.emplace_back(e.x(), e.y());
    }
  }

  std::vector<StaticObstacle> obstacles_;
  geo::Datum datum_;
  StaticObstacleParams params_;
  std::vector<Eigen::Vector2d> enu_;
};

}  // namespace navtracker
