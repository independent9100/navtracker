#pragma once

#include <string_view>
#include <vector>

#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"
#include "core/types/Measurement.hpp"
#include "ports/IHeadingBiasProvider.hpp"
#include "ports/ISensorAdapter.hpp"

namespace navtracker {

struct ArpaAdapterConfig {
  double heading_std_deg{0.0};
};

// Parses NMEA 0183 TTM (range/bearing) and TLL (target lat/lon) into
// Position2D Measurements in the supplied Datum's ENU frame. TTM needs
// the latest own-ship pose to project relative measurements.
class ArpaAdapter : public ISensorAdapter {
 public:
  ArpaAdapter(geo::Datum datum, OwnShipProvider& own_ship,
              ArpaAdapterConfig cfg = {},
              const IHeadingBiasProvider* bias_provider = nullptr);

  bool ingest(std::string_view line, Timestamp t);
  std::vector<Measurement> poll() override;

 private:
  geo::Datum datum_;
  OwnShipProvider& own_ship_;
  ArpaAdapterConfig cfg_;
  const IHeadingBiasProvider* bias_provider_;
  std::vector<Measurement> buffer_;
};

}  // namespace navtracker
