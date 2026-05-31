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

 private:
  OwnShipProvider& provider_;
};

}  // namespace navtracker
