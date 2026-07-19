// Minimal navtracker consumer — proves find_package(navtracker) + the exported
// navtracker::navtracker_core target compile, link, and run against an INSTALLED
// navtracker. This is the W6.4 install/export smoke test (docs/integration-guide
// §1). Build instructions are in this directory's README.md.
#include <iostream>

#include <Eigen/Core>

#include "core/geo/Datum.hpp"

int main() {
  navtracker::geo::Datum datum(navtracker::geo::Geodetic{53.5, 8.0, 0.0});
  const Eigen::Vector3d enu =
      datum.toEnu(navtracker::geo::Geodetic{53.51, 8.0, 0.0});
  std::cout << "navtracker find_package smoke OK: north=" << enu.y() << " m\n";
  return 0;
}
