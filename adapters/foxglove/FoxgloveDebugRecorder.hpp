// FoxgloveDebugRecorder — offline MCAP debug recorder for the navtracker
// fusion pipeline.  Implements ITrackSnapshotSink / ITrackSink /
// IInnovationSink / ICollisionRiskSink / IDatumChangeSink and two
// input-side taps (recordMeasurement, recordOwnShip).  Open the output
// .mcap in Lichtblick or Foxglove Studio for a scrubbable spatial view
// of tracks, detections, gates, associations, NIS, bias, and CPA.
//
// Usage and channel table: docs/debug-visualization.md
// How to read covariance vs gate ellipses, association lines, and NIS
// plots: docs/learning/11-gating-gnn-hungarian.md §9
#pragma once
#include <memory>
#include <string>
#include <unordered_map>
#include <Eigen/Core>
#include "ports/ITrackSnapshotSink.hpp"
#include "ports/ITrackSink.hpp"
#include "ports/IInnovationSink.hpp"
#include "ports/ICollisionRiskSink.hpp"
#include "ports/ISensorBiasProvider.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"     // OwnShipPose + IDatumChangeSink
#include "core/geo/Datum.hpp"
#include "core/types/Measurement.hpp"
#include "adapters/foxglove/McapWriter.hpp"

namespace navtracker::foxglove {

struct RecorderConfig {
  double ellipse_k = 2.0;       // confidence multiplier for covariance ellipses
  double gate_gamma = 0.0;      // chi-square gate threshold; 0 disables /gates ellipses
  double entity_lifetime_sec = 0.0;  // SceneUpdate entity lifetime; 0 = persist forever,
                                     // >0 auto-expires stale entities (clean "now" view)
};

class FoxgloveDebugRecorder final
    : public ITrackSnapshotSink, public ITrackSink, public IInnovationSink,
      public ICollisionRiskSink, public IDatumChangeSink {
 public:
  FoxgloveDebugRecorder(const std::string& path, const geo::Datum& datum,
                        const ISensorBiasProvider* bias = nullptr,
                        RecorderConfig cfg = {});
  ~FoxgloveDebugRecorder() override;

  // Input-side taps (called from app composition root).
  void recordMeasurement(const Measurement& m);
  void recordOwnShip(const OwnShipPose& pose);

  // ITrackSnapshotSink
  void onTracks(const std::vector<Track>& tracks, Timestamp now) override;
  // ITrackSink
  void onTrackInitiated(const TrackLifecycleEvent& e) override;
  void onTrackConfirmed(const TrackLifecycleEvent& e) override;
  void onTrackUpdated(const TrackLifecycleEvent& e) override;
  void onTrackDeleted(const TrackLifecycleEvent& e) override;
  // IInnovationSink
  void onInnovation(const InnovationEvent& e) override;
  // ICollisionRiskSink
  void onCollisionRisk(const CollisionRiskEvent& e) override;
  // IDatumChangeSink
  void onDatumRecentered(const geo::Datum& old_d, const geo::Datum& new_d) override;

  void close();

 private:
  void registerChannels();
  // Publish a one-time identity map->enu transform so the 3D panel has a frame
  // to anchor the enu-framed entities to (the Map panel needs no transform).
  void ensureRootFrame(Timestamp t);
  bool root_frame_done_{false};
  std::unique_ptr<McapWriter> w_;
  geo::Datum datum_;
  const ISensorBiasProvider* bias_;
  RecorderConfig cfg_;
  // Latest predicted innovation covariance per track (for /gates).
  std::unordered_map<std::uint64_t, Eigen::MatrixXd> last_S_;
  // Most recent event time seen (used by onDatumRecentered for the log msg).
  Timestamp last_time_{};
};

}  // namespace navtracker::foxglove
