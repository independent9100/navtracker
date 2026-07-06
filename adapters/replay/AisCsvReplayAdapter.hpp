#pragma once

#include <string>
#include <vector>

#include "core/geo/Datum.hpp"
#include "core/types/Measurement.hpp"

namespace navtracker::replay {

/**
 * Load a decoded AIS CSV (Philos / MarineCadastre / DMA / Stone-Soup-Solent
 * schema) into a vector of Measurement(Position2D) sorted by time. Each
 * row becomes one absolute-position fix in the supplied datum's ENU
 * frame, tagged with MMSI as an AssociationHint.
 *
 * Schemas auto-detected from the header row (case-insensitive):
 *   Philos:        unix_time,mmsi,lat,lon,...
 *   MarineCadastre: MMSI,BaseDateTime,LAT,LON,SOG,...
 *   Stone Soup:     Time,MMSI,Latitude_degrees,Longitude_degrees,...
 *   DMA:            # Timestamp,Type of mobile,MMSI,Latitude,Longitude,...
 *
 * `tod` reading:
 *   Unix-epoch float (Philos) -> Timestamp::fromSeconds
 *   ISO 8601 string           -> parsed (T or space separator, optional
 *                                 fractional seconds; year-month-day
 *                                 HMS to nanoseconds-since-1970)
 *
 * `sigma` for the 2x2 R is fixed at 30 m here — AIS class-A nominal
 * "low accuracy" position. Class-B and high-accuracy override could be
 * added later when fields are populated.
 *
 * `emit_velocity` (default OFF → historical byte-identical Position2D output):
 * when true AND the CSV carries `sog`/`cog` columns, each fix above the SOG
 * threshold becomes a PositionVelocity2D measurement using the SAME polar
 * SOG/COG → velocity math as the NMEA AisAdapter (backlog #20 increment 2 —
 * see core/estimation/PolarVelocity.hpp; the two paths share one helper so they
 * cannot drift). A `nav_status` column, when present, is surfaced on
 * `hints.nav_status` (feeds the anchored-veto path, ADR 0002 / R3). Columns are
 * detected case-insensitively (`sog`/`sog_mps`, `cog`/`cog_deg`, `nav_status`);
 * their absence is tolerated (falls back to Position2D). Off is the default so
 * every existing call site stays bit-identical.
 */
std::vector<Measurement> loadAisCsv(const std::string& path,
                                    const geo::Datum& datum,
                                    std::string source_id,
                                    bool emit_velocity = false);

}  // namespace navtracker::replay
