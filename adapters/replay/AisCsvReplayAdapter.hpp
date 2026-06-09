#pragma once

#include <string>
#include <vector>

#include "core/geo/Datum.hpp"
#include "core/types/Measurement.hpp"

namespace navtracker::replay {

// Load a decoded AIS CSV (Philos / MarineCadastre / DMA / Stone-Soup-Solent
// schema) into a vector of Measurement(Position2D) sorted by time. Each
// row becomes one absolute-position fix in the supplied datum's ENU
// frame, tagged with MMSI as an AssociationHint.
//
// Schemas auto-detected from the header row (case-insensitive):
//   Philos:        unix_time,mmsi,lat,lon,...
//   MarineCadastre: MMSI,BaseDateTime,LAT,LON,SOG,...
//   Stone Soup:     Time,MMSI,Latitude_degrees,Longitude_degrees,...
//   DMA:            # Timestamp,Type of mobile,MMSI,Latitude,Longitude,...
//
// `tod` reading:
//   Unix-epoch float (Philos) -> Timestamp::fromSeconds
//   ISO 8601 string           -> parsed (T or space separator, optional
//                                 fractional seconds; year-month-day
//                                 HMS to nanoseconds-since-1970)
//
// `sigma` for the 2x2 R is fixed at 30 m here — AIS class-A nominal
// "low accuracy" position. Class-B and high-accuracy override could be
// added later when fields are populated.
std::vector<Measurement> loadAisCsv(const std::string& path,
                                    const geo::Datum& datum,
                                    std::string source_id);

}  // namespace navtracker::replay
