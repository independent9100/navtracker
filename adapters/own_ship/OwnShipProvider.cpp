#include "adapters/own_ship/OwnShipProvider.hpp"

namespace navtracker {

void OwnShipProvider::update(const OwnShipPose& pose) { latest_ = pose; }
std::optional<OwnShipPose> OwnShipProvider::latest() const { return latest_; }

}  // namespace navtracker
