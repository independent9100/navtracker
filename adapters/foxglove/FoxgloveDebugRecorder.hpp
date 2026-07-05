#pragma once
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <Eigen/Core>
#include "ports/ITrackSnapshotSink.hpp"
#include "ports/ITrackSink.hpp"
#include "ports/IInnovationSink.hpp"
#include "ports/ICollisionRiskSink.hpp"
#include "ports/IStaticHazardSink.hpp"
#include "ports/ISensorBiasProvider.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"     // OwnShipPose + IDatumChangeSink
#include "core/geo/Datum.hpp"
#include "core/types/Measurement.hpp"
#include "adapters/foxglove/McapWriter.hpp"

namespace navtracker {
// Domain types drawn by the environment/PMBM taps — included in the .cpp; only
// referenced here by const-ref, so forward declarations keep the header light.
struct StaticObstacle;
struct LandPolygon;
class LiveOccupancyModel;
class ClutterMapSensorDetectionModel;
namespace pmbm { struct PmbmDensity; }
}  // namespace navtracker

namespace navtracker::foxglove {

/** Tuning knobs for the debug recorder's visual output (ellipse scale,
 *  gate threshold, entity lifetime). */
struct RecorderConfig {
  // Master on/off switch for ALL debug drawing (old and new layers). When
  // false every recorder entry point early-returns and nothing is written.
  // This is a per-instance member threaded through the constructor — never a
  // global / static / singleton — so it is safe across dynamic-library
  // boundaries and multiple recorder instances.
  bool enabled = true;
  double ellipse_k = 2.0;       // confidence multiplier for covariance ellipses
  double gate_gamma = 0.0;      // chi-square gate threshold; 0 disables /gates ellipses
  double entity_lifetime_sec = 0.0;  // SceneUpdate entity lifetime; 0 = persist forever,
                                     // >0 auto-expires stale entities (clean "now" view)
};

/**
 * FoxgloveDebugRecorder — offline MCAP debug recorder for the navtracker
 * fusion pipeline.  Implements ITrackSnapshotSink / ITrackSink /
 * IInnovationSink / ICollisionRiskSink / IDatumChangeSink and two
 * input-side taps (recordMeasurement, recordOwnShip).  Open the output
 * .mcap in Lichtblick or Foxglove Studio for a scrubbable spatial view
 * of tracks, detections, gates, associations, NIS, bias, and CPA.
 *
 * Usage and channel table: docs/debug-visualization.md
 * How to read covariance vs gate ellipses, association lines, and NIS
 * plots: docs/learning/11-gating-gnn-hungarian.md §9
 */
class FoxgloveDebugRecorder final
    : public ITrackSnapshotSink, public ITrackSink, public IInnovationSink,
      public ICollisionRiskSink, public IStaticHazardSink, public IDatumChangeSink {
 public:
  /** Open `path` for writing; entities are placed in the ENU frame anchored
   *  at `datum`. Optional `bias` provider is sampled for the /bias diagnostic
   *  channel. `cfg` tunes ellipse/gate/lifetime rendering. */
  FoxgloveDebugRecorder(const std::string& path, const geo::Datum& datum,
                        const ISensorBiasProvider* bias = nullptr,
                        RecorderConfig cfg = {});
  ~FoxgloveDebugRecorder() override;

  // Input-side taps (called from app composition root).
  /** Record an incoming sensor measurement (detection marker + gate/wedge). */
  void recordMeasurement(const Measurement& m);
  /** Record an own-ship pose (position track + heading). */
  void recordOwnShip(const OwnShipPose& pose);

  // Environment / static-world taps (called from the composition root).
  /** Draw the coastline land polygons (outer ring + holes) as outlines. Static;
   *  call once (or whenever the geometry changes). Topic: /land. */
  void recordCoastline(const std::vector<LandPolygon>& polys);
  /** Draw charted static obstacles: centre + footprint + keep-clear rings.
   *  Static; call once / on change. Topic: /static_obstacles. */
  void recordStaticObstacles(const std::vector<StaticObstacle>& obstacles);
  /** Draw the live-occupancy state at time `now`: persistence heatmap, learned
   *  structure hazards, charted points, camera-empty cells, vessel-fix veto
   *  rings. Per scan. Topics under /occupancy. */
  void recordOccupancy(const LiveOccupancyModel& occ, Timestamp now);

  // PMBM posterior tap (called from the composition root, per scan).
  /** Draw the PMBM posterior at time `now`: PPP intensity ellipses, Bernoulli
   *  existence ellipses, and per-target trajectories. Topics under /pmbm. */
  void recordPmbmDensity(const pmbm::PmbmDensity& density, Timestamp now);

  // Sensor-modelling tap (per scan or once for declared coverage).
  /** Draw a sensor coverage sector/disc at `sensor_enu` (ENU bearing
   *  `center_rad`, half-width `half_width_rad`, out to `range_m`). Topic:
   *  /coverage/<source_id>. */
  void recordSensorCoverage(const std::string& source_id, SensorKind sensor,
                            const Eigen::Vector2d& sensor_enu, double center_rad,
                            double half_width_rad, double range_m, Timestamp now);
  /** Draw the learned clutter-intensity map: a position-space heatmap and, when
   *  the bearing map is enabled, an azimuth rose centred at `origin_enu` (radial
   *  segments, length ∝ normalized λ). Topics under /clutter. */
  void recordClutterMap(const ClutterMapSensorDetectionModel& clutter,
                        const Eigen::Vector2d& origin_enu, Timestamp now);

  // ITrackSnapshotSink
  /** Write the current set of tracks as a SceneUpdate at time `now`. */
  void onTracks(const std::vector<Track>& tracks, Timestamp now) override;
  // ITrackSink
  /** Log a track-initiation lifecycle event. */
  void onTrackInitiated(const TrackLifecycleEvent& e) override;
  /** Log a track-confirmation lifecycle event. */
  void onTrackConfirmed(const TrackLifecycleEvent& e) override;
  /** Log a track-update lifecycle event. */
  void onTrackUpdated(const TrackLifecycleEvent& e) override;
  /** Log a track-deletion lifecycle event. */
  void onTrackDeleted(const TrackLifecycleEvent& e) override;
  // IInnovationSink
  /** Record a per-update innovation (NIS plot + cached S for /gates). */
  void onInnovation(const InnovationEvent& e) override;
  // ICollisionRiskSink
  /** Record a CPA collision-risk event (Entered/Exited/Updated). */
  void onCollisionRisk(const CollisionRiskEvent& e) override;
  // IStaticHazardSink
  /** Record a static-hazard keep-clear crossing (log + marker). */
  void onStaticHazard(const StaticHazardEvent& e) override;
  // IDatumChangeSink
  /** React to a datum recenter; logs the shift for downstream frame context. */
  void onDatumRecentered(const geo::Datum& old_d, const geo::Datum& new_d) override;

  /** Flush and close the underlying MCAP writer. */
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
