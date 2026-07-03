#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "core/own_ship/OwnShipProvider.hpp"
#include "core/types/Measurement.hpp"

namespace navtracker::replay {

// Diagnostic counts from a camera_bearings.csv load.
struct CameraBearingLoadStats {
  std::size_t rows_read{0};
  std::size_t emitted{0};
  std::size_t dropped_no_pose{0};   // no own-ship pose at-or-before the row
  std::size_t dropped_invalid{0};   // non-finite / non-positive sigma / bad row
};

// Load the philos camera-bearing fixture into bearing-only Measurements.
//
// Fixture schema (see tests/fixtures/philos/README.md):
//   unix_time,camera,bearing_rel_deg,sigma_deg,confidence,u_px,v_px,w_px,h_px,frame
//
// `bearing_rel_deg` is HULL-RELATIVE (0 = bow, clockwise/starboard +). This
// loader composes own-ship heading at load time — mirroring the TTM-over-TLL
// rule (docs/sensors/sensor-reference.md §2) so heading-bias machinery can
// apply downstream — to produce an absolute ENU Bearing2D measurement:
//
//   marine_true = pose.heading_true_deg + bearing_rel_deg     (deg, N=0 CW)
//   value(0)    = wrapAngle(pi/2 - deg2rad(marine_true))       (= atan2(dN,dE))
//   R(0,0)      = (sigma_deg * pi/180)^2                        (rad^2)
//   sensor_position_enu = provider.datum().toEnu(pose lat/lon)
//   sensor = EoIr;  source_id = source_prefix + "_" + camera
//   model  = Bearing2D  (canInitiateTrack == false: corroborate, never birth)
//
// The `value(0)` convention is identical to how loadPlotCsvBodyFrame places
// radar plots (Projection.cpp: east = r*sin(marine), north = r*cos(marine)),
// so a camera bearing and a radar plot of the same object share the ENU frame.
//
// The `OwnShipProvider` must be pre-populated (e.g. via feedOwnshipHistory)
// so its datum is established and poseAtOrBefore(t) resolves. Rows with no
// available pose at-or-before their timestamp are dropped and counted.
std::vector<Measurement> loadCameraBearingsCsv(
    const std::string& path,
    const OwnShipProvider& provider,
    std::string source_prefix = "philos_cam",
    CameraBearingLoadStats* stats = nullptr);

}  // namespace navtracker::replay
