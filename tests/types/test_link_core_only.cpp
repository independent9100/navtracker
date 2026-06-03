#include "core/types/SensorDefaults.hpp"
#include <iostream>

int main() {
  const auto d = navtracker::pessimisticSensorDefaults();
  std::cout << "core-only link OK; AIS sigma_pos=" << d.ais_position.sigma_pos_m << "\n";
  return 0;
}
