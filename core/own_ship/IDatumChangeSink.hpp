#pragma once

#include "core/geo/Datum.hpp"

namespace navtracker {

/**
 * Notified when OwnShipProvider replaces its working datum (e.g. when
 * the ship has moved far enough that the local tangent plane needs to
 * be re-anchored). Consumers that hold state in ENU coordinates should
 * react to this event to keep their state consistent with the new frame.
 *
 * Sensor adapters implement this too (W2.1): on recenter they swap in the
 * new datum and re-express any buffered measurements — see the adapter
 * headers and docs/integration-guide.md §"Auto-datum".
 */
class IDatumChangeSink {
 public:
  virtual ~IDatumChangeSink() = default;
  virtual void onDatumRecentered(const geo::Datum& old_datum,
                                 const geo::Datum& new_datum) = 0;
};

}  // namespace navtracker
