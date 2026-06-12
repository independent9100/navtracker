#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "core/types/Ids.hpp"
#include "core/types/Measurement.hpp"
#include "core/types/Timestamp.hpp"
#include "ports/ISensorDetectionModel.hpp"

namespace navtracker {

// Tuning for the spatial clutter map (see class doc below).
struct ClutterMapParams {
  // ENU grid pitch for position-space maps (Position2D /
  // PositionVelocity2D). Cell area = cell_size_m².
  double cell_size_m{50.0};
  // Bearing-space maps (Bearing2D) are OFF by default. Bearings cannot
  // initiate tracks, so a real target whose track has lapsed keeps
  // feeding "unassociated" bearings at its own azimuth: the map raises
  // λ exactly where the target is, blocks re-confirmation, and the
  // suppression self-reinforces. Measured on AutoFerry (2026-06-12):
  // bearing map alone collapses lifetime sc17 0.90 → 0.28, sc5
  // 0.91 → 0.34 while the position map alone is lifetime-neutral.
  // Opt in only when tracks can be born from bearings or the clutter
  // proxy can exclude trackless targets.
  bool enable_bearing_map{false};
  // Azimuth pitch for bearing-space maps; the circle is divided into
  // round(2π / bearing_cell_rad) equal cells. Default 5°.
  double bearing_cell_rad{0.0872664625997164788};
  // EWMA memory τ in SECONDS. Time-based, never scan-counted — the same
  // map behaves identically whether the sensor runs at 16 Hz or 0.1 Hz.
  double time_constant_s{20.0};
  // Effective age of the baseline prior when a cell is first touched:
  // the first observation gets weight w₀ = 1 − exp(−prior_dt_s/τ), so a
  // single spurious return moves the cell but cannot replace the prior.
  double prior_dt_s{5.0};
  // The resolved λ is clamped to [baseline·min_ratio, baseline·max_ratio]
  // where baseline is the wrapped table's (source-aware) λ for the
  // queried measurement. Ratios (not absolute densities) so the clamp is
  // dimensionally sane across m⁻² and rad⁻¹ sensors simultaneously.
  double min_ratio{0.125};
  double max_ratio{64.0};
};

// Spatially varying clutter intensity — a decorator over a fixed
// per-sensor detection table (backlog item 5).
//
// ## Math
// Per (SensorKind, MeasurementModel) the model keeps a sparse grid of
// cells; each cell c holds an EWMA estimate r_c of "unassociated returns
// per scan landing in c" plus the time of its last touch:
//
//   touch at time t with count n:
//     w   = 1 − exp(−(t − t_last)/τ)        (first touch: Δt = prior_dt_s)
//     r_c ← (1 − w)·r_c + w·n,   t_last ← t
//
// A cell is touched on every scan in which ANY return (associated or
// not) lands in it: associated traffic contributes n = 0, dragging the
// cell toward zero — evidence the sensor surveys the area and its
// returns are targets, not clutter. Cells never receiving returns are
// never created and read back as the table baseline (prior; absence of
// evidence is not evidence of absence).
//
// Query (the virtual paramsFor(Measurement) override):
//   λ_c        = r_c / A          (A = cell_size² m², or cell width rad)
//   λ(z)       = interpolation of λ_c at z's position — bilinear over
//                the 4 surrounding cell centers (position maps), linear
//                over the 2 adjacent centers with ±π wraparound
//                (bearing maps); absent cells contribute the baseline.
//   λ_resolved = clamp(λ(z), baseline·min_ratio, baseline·max_ratio)
// P_D, max_range_m, and sector fields always come from the wrapped
// table (source-keyed lookups included); only clutter_intensity is
// position-resolved.
//
// Map spaces: Position2D / PositionVelocity2D → 2-D ENU grid (λ in
// m⁻²); Bearing2D → 1-D circular azimuth grid in absolute ENU azimuth
// (λ in rad⁻¹). RangeBearing2D ((m·rad)⁻¹) has no map yet and passes
// through.
//
// ## Assumptions
// - Unassociated returns ("gated to no existing track") are a usable
//   clutter proxy. Persistent structure that births its own clutter
//   track becomes "associated" one scan later, so the proxy
//   *undercounts* stable shoreline structure for position sensors;
//   bearings can't initiate tracks and keep counting.
// - Clutter fields move slowly relative to τ (shorelines do; rain
//   squalls marginal).
// - Bearing clutter is usefully indexed by absolute azimuth from
//   ownship — valid while ownship moves slowly relative to τ.
//
// ## Rationale
// Both real datasets (AutoFerry urban channel, philos Boston harbour)
// show the same failure: clutter is persistent *structured* shoreline
// return, not uniform Poisson, and feeding its ML-fitted rate into the
// uniform-λ score collapsed true-track lifetime (measured, evaluation
// log 2026-06-11). A clutter map keeps the calibrated baseline where
// nothing has been learned and raises λ only where false returns
// actually recur — the standard clutter-map CFAR idea transplanted to
// the MHT score. A decorator (vs a new table type) keeps the fixed
// path bit-identical when unwrapped and the hot path untouched: the
// score already calls paramsFor(z).
//
// ## Ways to improve / what to test next
// - Use global-hypothesis association (not the birth-gate proxy) to
//   label clutter; would catch position-sensor shoreline structure.
// - Per-source maps (EO vs IR) once per-source clutter measurably
//   differs; currently kind-wide like the backlog specifies.
// - Range–bearing product space map for RangeBearing2D sensors.
// - Forgetting toward the prior for cells unvisited ≫ τ (currently
//   they keep their last estimate — persistent map memory).
class ClutterMapSensorDetectionModel : public ISensorDetectionModel {
 public:
  explicit ClutterMapSensorDetectionModel(
      std::shared_ptr<ISensorDetectionModel> inner,
      ClutterMapParams params = {});

  using ISensorDetectionModel::paramsFor;
  DetectionParams paramsFor(SensorKind sensor,
                            MeasurementModel model) const override;
  DetectionParams paramsFor(SensorKind sensor, MeasurementModel model,
                            const std::string& source_id) const override;
  DetectionParams paramsFor(const Measurement& z) const override;

  void observe(const std::vector<ScanObservation>& by_sensor) override;

 private:
  struct Cell {
    double rate{0.0};  // EWMA of unassociated returns per scan in cell
    Timestamp last;
  };
  using MapKey = std::pair<SensorKind, MeasurementModel>;
  struct PositionMap {
    std::map<std::pair<std::int64_t, std::int64_t>, Cell> cells;
  };
  struct BearingMap {
    std::vector<Cell> cells;     // dense circular grid
    std::vector<bool> touched;   // cell ever observed
  };

  // λ_c of one position cell at index (ix, iy): cell rate / area, or
  // the baseline for never-touched cells.
  double positionCellLambda(const PositionMap& m,
                            std::pair<std::int64_t, std::int64_t> idx,
                            double baseline) const;
  double bearingCellLambda(const BearingMap& m, std::size_t idx,
                           double baseline) const;

  std::shared_ptr<ISensorDetectionModel> inner_;
  ClutterMapParams p_;
  std::size_t bearing_cell_count_;
  std::map<MapKey, PositionMap> position_maps_;
  std::map<MapKey, BearingMap> bearing_maps_;
};

}  // namespace navtracker
