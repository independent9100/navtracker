#pragma once

namespace navtracker::edge {

// Edge validation helpers shared by the live sensor adapters. Invariant
// #6 (CLAUDE.md) requires adapters to reject implausible / NaN input at
// the boundary so the domain core can trust its inputs. These are the
// single source of truth for "is this reading usable"; the replay
// adapters apply the same lat/lon range check inline.

// True iff a geodetic position is plausible WGS84: both coordinates
// finite and within [-90, 90] / [-180, 180]. This rejects AIS
// "position not available" sentinels (lat 91°, lon 181°) and NaN/Inf.
bool isPlausibleLatLon(double lat_deg, double lon_deg);

// True iff a range reading is usable: finite and strictly positive.
// Rejects strtod parse-failure 0.0 (which would collapse a target onto
// own-ship) and negative / NaN / Inf ranges.
bool isPlausibleRange(double range_m);

// True iff v is finite (not NaN / Inf). Used for raw bearings.
bool isFiniteValue(double v);

}  // namespace navtracker::edge
