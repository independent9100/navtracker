#pragma once

#include <Eigen/Core>

#include "core/geo/Datum.hpp"
#include "core/land/CoastlineGeometry.hpp"
#include "core/own_ship/OwnShipProvider.hpp"  // IDatumChangeSink
#include "ports/ILandModel.hpp"

namespace navtracker {

/**
 * Concrete land model: holds coastline geometry (geodetic) + the working datum.
 * clutterPrior converts the ENU query to geodetic via the current datum, so a
 * datum recenter is just a datum swap (no polygon reprojection). Pure: no I/O,
 * no wall-clock, no RNG. setCoastline / onDatumRecentered must be invoked at
 * deterministic, timestamp-ordered points (see spec §5).
 *
 * Datum storage: geo::Datum stores only value members (Geodetic + two Eigen
 * matrices), so the compiler-generated copy-assignment operator is usable.
 * We therefore store `geo::Datum datum_` directly and assign in
 * onDatumRecentered — no std::optional needed.
 */
class CoastlineModel : public ILandModel, public IDatumChangeSink {
 public:
  CoastlineModel(CoastlineGeometry geom, geo::Datum datum)
      : geom_(std::move(geom)), datum_(datum) {}

  /**
   * ILandModel: clutter prior at an ENU query point. Converts to geodetic via
   * the current datum, then evaluates the coastline geometry there.
   */
  double clutterPrior(const Eigen::Vector2d& enu_xy) const override {
    const geo::Geodetic g =
        datum_.toGeodetic(Eigen::Vector3d(enu_xy.x(), enu_xy.y(), 0.0));
    return geom_.priorAtGeodetic(g.lat_deg, g.lon_deg);
  }

  /** IDatumChangeSink: swap in the new datum so subsequent ENU→geodetic queries stay correct. */
  void onDatumRecentered(const geo::Datum& /*old_datum*/,
                         const geo::Datum& new_datum) override {
    datum_ = new_datum;
  }

  /** Replace the coastline geometry (must be called at a deterministic, timestamp-ordered point). */
  void setCoastline(CoastlineGeometry geom) { geom_ = std::move(geom); }

 private:
  CoastlineGeometry geom_;
  geo::Datum datum_;
};

}  // namespace navtracker
