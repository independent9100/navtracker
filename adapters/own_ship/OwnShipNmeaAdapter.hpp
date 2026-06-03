#pragma once

#include <string_view>

#include "adapters/own_ship/OwnShipProvider.hpp"

namespace navtracker {

// Parses NMEA 0183 GGA (position) and HDT (true heading) into OwnShipPose
// updates on the supplied OwnShipProvider. The caller supplies a full
// Timestamp per ingest.
class OwnShipNmeaAdapter {
 public:
  explicit OwnShipNmeaAdapter(OwnShipProvider& provider);

  bool ingest(std::string_view line, Timestamp t);

  // Sim hook: sets the position uncertainty applied to every subsequently
  // published pose. Sticky until overwritten. Real NMEA paths (Task 3) will
  // derive this internally from GGA HDOP × UERE.
  void setPositionStd(double sigma_m);

 private:
  OwnShipProvider& provider_;
  double position_std_m_{0.0};
};

}  // namespace navtracker
