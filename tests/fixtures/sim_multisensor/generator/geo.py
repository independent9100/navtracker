"""WGS-84 local-tangent ENU datum, bit-compatible with navtracker's C++.

This replicates ``core/geo/Wgs84.cpp`` (geodeticToEcef / ecefToGeodetic) and
``core/geo/Datum.cpp`` (ENU rotation) EXACTLY, using the same constants and the
same rotation matrix. This is a hard requirement, not a nicety: the C++ bench
projects both the truth CSV (lat/lon -> ENU) and the AIS/radar measurements
(lat/lon or own-ship-relative range/az -> ENU) through ``geo::Datum``. If the
Python transform used to author range/azimuth and lat/lon disagreed with the
C++ one, radar plots would systematically miss the truth and the "accuracy"
gate would measure a projection bug instead of the tracker.

Angle conventions
-----------------
* Geodetic: degrees.
* Marine bearing / COG / heading: degrees, North = 0, clockwise positive
  (this is what every navtracker CSV column uses at the edges).
* ENU: metres, x = East, y = North. A marine bearing ``b`` points along
  ``(sin b, cos b)`` in ENU.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# --- WGS-84 constants (identical to core/geo/Wgs84.cpp) ----------------------
_A = 6378137.0                      # semi-major axis (m)
_F = 1.0 / 298.257223563            # flattening
_E2 = _F * (2.0 - _F)               # first eccentricity squared
_B = _A * (1.0 - _F)                # semi-minor axis (m)
_EP2 = (_A * _A - _B * _B) / (_B * _B)  # second eccentricity squared


def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float = 0.0) -> np.ndarray:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    s = math.sin(lat)
    c = math.cos(lat)
    n = _A / math.sqrt(1.0 - _E2 * s * s)
    x = (n + alt_m) * c * math.cos(lon)
    y = (n + alt_m) * c * math.sin(lon)
    z = (n * (1.0 - _E2) + alt_m) * s
    return np.array([x, y, z], dtype=float)


def ecef_to_geodetic(ecef: np.ndarray) -> tuple[float, float, float]:
    x, y, z = float(ecef[0]), float(ecef[1]), float(ecef[2])
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    theta = math.atan2(z * _A, p * _B)
    st = math.sin(theta)
    ct = math.cos(theta)
    lat = math.atan2(z + _EP2 * _B * st ** 3, p - _E2 * _A * ct ** 3)
    sl = math.sin(lat)
    n = _A / math.sqrt(1.0 - _E2 * sl * sl)
    alt = p / math.cos(lat) - n
    return math.degrees(lat), math.degrees(lon), alt


@dataclass(frozen=True)
class Datum:
    """Local ENU tangent plane about a geodetic origin (mirrors geo::Datum)."""

    lat0_deg: float
    lon0_deg: float
    alt0_m: float = 0.0

    def _rot(self) -> np.ndarray:
        lat = math.radians(self.lat0_deg)
        lon = math.radians(self.lon0_deg)
        s_lat, c_lat = math.sin(lat), math.cos(lat)
        s_lon, c_lon = math.sin(lon), math.cos(lon)
        return np.array([
            [-s_lon,          c_lon,          0.0],
            [-s_lat * c_lon, -s_lat * s_lon,  c_lat],
            [ c_lat * c_lon,  c_lat * s_lon,  s_lat],
        ], dtype=float)

    def _origin_ecef(self) -> np.ndarray:
        return geodetic_to_ecef(self.lat0_deg, self.lon0_deg, self.alt0_m)

    def to_enu(self, lat_deg: float, lon_deg: float, alt_m: float = 0.0) -> np.ndarray:
        """Geodetic -> ENU (metres). Matches Datum::toEnu."""
        return self._rot() @ (geodetic_to_ecef(lat_deg, lon_deg, alt_m) - self._origin_ecef())

    def to_geodetic(self, enu: np.ndarray) -> tuple[float, float, float]:
        """ENU (metres) -> geodetic. Matches Datum::toGeodetic."""
        ecef = self._rot().T @ np.asarray(enu, dtype=float) + self._origin_ecef()
        return ecef_to_geodetic(ecef)

    def enu_to_lonlat(self, east_m: float, north_m: float) -> tuple[float, float]:
        """Convenience: 2-D ENU (E, N) -> (lat_deg, lon_deg)."""
        lat, lon, _ = self.to_geodetic(np.array([east_m, north_m, 0.0]))
        return lat, lon


# --- marine angle helpers ----------------------------------------------------
def wrap_deg_0_360(deg: float) -> float:
    """Wrap to [0, 360)."""
    return deg % 360.0


def wrap_deg_180(deg: float) -> float:
    """Wrap to (-180, 180]."""
    d = (deg + 180.0) % 360.0 - 180.0
    return d + 360.0 if d <= -180.0 else d


def marine_bearing_from_enu(d_east: float, d_north: float) -> float:
    """ENU delta -> marine bearing (deg, N=0 CW). Inverse of (sin,cos)."""
    return wrap_deg_0_360(math.degrees(math.atan2(d_east, d_north)))


def enu_from_marine(distance_m: float, bearing_deg: float) -> tuple[float, float]:
    """Marine range/bearing -> ENU (E, N)."""
    b = math.radians(bearing_deg)
    return distance_m * math.sin(b), distance_m * math.cos(b)
