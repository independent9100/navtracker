#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "core/geo/Datum.hpp"

namespace navtracker {

struct BearingWedge;  // core/static/BearingWedgeModel.hpp

/**
 * Drainable output for one bearing-wedge hazard (backlog #17 option 1), parallel
 * to StaticHazardOutput. A wedge is a DIRECTION from own-ship, not a position:
 * the apex is own-ship's geodetic position at detection, and the hazard spans
 * ±half_width_deg about bearing_true_deg out to max_range_m (unbounded when
 * absent). CPA is not computable (no target position); the operator reading is
 * "keep clear along that line". is_charted is always false (live-detected).
 */
struct BearingWedgeOutput {
  std::uint64_t hazard_id{0};
  double apex_lat_deg{0.0};
  double apex_lon_deg{0.0};
  double bearing_true_deg{0.0};       // CW from true north, [0, 360)
  double half_width_deg{0.0};
  std::optional<double> max_range_m;  // nullopt ⇒ range unbounded/unknown
  bool is_charted{false};
  std::string source_id;
};

/**
 * Build a BearingWedgeOutput from a wedge, converting the anchor-frame apex to
 * geodetic via `anchor` and the internal math bearing to a true bearing.
 */
BearingWedgeOutput toBearingWedgeOutput(const BearingWedge& wedge,
                                        const geo::Datum& anchor);

}  // namespace navtracker
