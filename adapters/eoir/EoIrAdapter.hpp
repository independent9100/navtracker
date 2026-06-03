#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/geo/Datum.hpp"
#include "core/types/Measurement.hpp"
#include "ports/IHeadingBiasProvider.hpp"
#include "ports/ISensorAdapter.hpp"

namespace navtracker {

struct CameraDetection {
  Timestamp time;
  double bearing_relative_deg{0.0};
  double range_m{0.0};
  double bearing_std_deg{0.5};
  double range_std_m{10.0};
  std::optional<std::int32_t> sensor_track_id;
  std::string source_id{"eo_ir"};
};

struct EoIrAdapterConfig {
  double heading_std_deg{0.0};
};

class EoIrAdapter : public ISensorAdapter {
 public:
  EoIrAdapter(geo::Datum datum, OwnShipProvider& own_ship,
              EoIrAdapterConfig cfg = {},
              const IHeadingBiasProvider* bias_provider = nullptr);

  void ingest(const CameraDetection& d);
  std::vector<Measurement> poll() override;

 private:
  geo::Datum datum_;
  OwnShipProvider& own_ship_;
  EoIrAdapterConfig cfg_;
  const IHeadingBiasProvider* bias_provider_;
  std::vector<Measurement> buffer_;
};

}  // namespace navtracker
