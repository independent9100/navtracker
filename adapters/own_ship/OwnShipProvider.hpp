#pragma once

#include <optional>

#include "core/types/Timestamp.hpp"

namespace navtracker {

struct OwnShipPose {
  Timestamp time;
  double lat_deg{0.0};
  double lon_deg{0.0};
  double alt_m{0.0};
  double heading_true_deg{0.0};
  double position_std_m{0.0};
};

class OwnShipProvider {
 public:
  void update(const OwnShipPose& pose);
  std::optional<OwnShipPose> latest() const;

 private:
  std::optional<OwnShipPose> latest_;
};

}  // namespace navtracker
