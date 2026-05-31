#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "core/geo/Datum.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Timestamp.hpp"
#include "ports/ISensorAdapter.hpp"

namespace navtracker {

struct AisDynamicReport {
  Timestamp time;
  std::uint32_t mmsi{0};
  double lat_deg{0.0};
  double lon_deg{0.0};
  bool high_accuracy{false};
  std::string source_id{"ais"};
};

class AisAdapter : public ISensorAdapter {
 public:
  explicit AisAdapter(geo::Datum datum);

  void ingest(const AisDynamicReport& r);
  std::vector<Measurement> poll() override;

 private:
  geo::Datum datum_;
  std::vector<Measurement> buffer_;
};

}  // namespace navtracker
