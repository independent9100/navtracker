#pragma once

#include <Eigen/Core>

#include "core/geo/AxisRotation.hpp"
#include "core/geo/Datum.hpp"
#include "core/types/Measurement.hpp"

namespace navtracker::adapter_util {

/**
 * Re-express one already-buffered ENU `Measurement` from `old_datum`'s frame
 * into `new_datum`'s frame, for use by a sensor adapter's IDatumChangeSink
 * handler when the working datum auto-recenters (W2.1).
 *
 * A buffered measurement was projected with the pre-recenter datum; without
 * this a single poll() after a recenter would hand the tracker a mix of old-
 * and new-frame measurements. The transform mirrors DatumShift exactly:
 *   - position (value[0..1]): exact old-ENU → geodetic → new-ENU round-trip;
 *   - velocity (value[2..3], PositionVelocity2D): rotate by datumAxisRotation;
 *   - covariance: rotate the position block (and the velocity block if 4×4)
 *     by the same R.
 *
 * `sensor_position_enu` is rotated/translated the same way ONLY when it is set
 * (non-zero); the default (0,0) "sensor at datum / unset" is left untouched so
 * the recenter does not fabricate a spurious sensor offset.
 */
inline void reprojectMeasurementEnu(Measurement& m, const geo::Datum& old_datum,
                                    const geo::Datum& new_datum) {
  if (m.value.size() < 2) return;
  const Eigen::Matrix2d R = geo::datumAxisRotation(old_datum, new_datum);

  // Position: exact round-trip through geodetic (matches DatumShift).
  const auto g =
      old_datum.toGeodetic(Eigen::Vector3d(m.value(0), m.value(1), 0.0));
  const Eigen::Vector3d p_new = new_datum.toEnu(g);
  m.value(0) = p_new.x();
  m.value(1) = p_new.y();

  // Velocity block (PositionVelocity2D): rotate.
  if (m.value.size() >= 4) {
    const Eigen::Vector2d v_new = R * Eigen::Vector2d(m.value(2), m.value(3));
    m.value(2) = v_new.x();
    m.value(3) = v_new.y();
  }

  // Covariance: rotate the position block, and the velocity block when present.
  if (m.covariance.rows() >= 2 && m.covariance.cols() >= 2) {
    m.covariance.topLeftCorner<2, 2>() =
        R * m.covariance.topLeftCorner<2, 2>().eval() * R.transpose();
  }
  if (m.covariance.rows() >= 4 && m.covariance.cols() >= 4) {
    m.covariance.block<2, 2>(2, 2) =
        R * m.covariance.block<2, 2>(2, 2).eval() * R.transpose();
  }

  // Sensor position: only when actually populated (see doc above).
  if (!m.sensor_position_enu.isZero()) {
    const auto gs = old_datum.toGeodetic(Eigen::Vector3d(
        m.sensor_position_enu.x(), m.sensor_position_enu.y(), 0.0));
    const Eigen::Vector3d s_new = new_datum.toEnu(gs);
    m.sensor_position_enu = Eigen::Vector2d(s_new.x(), s_new.y());
  }
}

}  // namespace navtracker::adapter_util
