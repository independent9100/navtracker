#include "core/pipeline/Tracker.hpp"

#include <cmath>
#include <cstdint>
#include <vector>

#include "core/estimation/MeasurementModels.hpp"
#include "core/tracking/TrackManager.hpp"

namespace navtracker {
namespace {

// `weight` is the JPDA β for this measurement (1.0 for hard matches). The
// effective observation variance is scaled by 1/weight to encode "this
// measurement only β-likely came from this track" — small β means the
// estimator's KF gain is small, equivalent to under-weighting the
// observation. Both state_var and R are scaled identically so the
// state-dominance gate ratio (and hence acceptance) is preserved.
void emitBearingInnovationIfApplicable(IBearingInnovationSink* sink,
                                       const Track& tr_pred,
                                       const Measurement& z,
                                       double weight = 1.0) {
  if (sink == nullptr) return;
  if (weight <= 0.0) return;
  if (z.model != MeasurementModel::Bearing2D &&
      z.model != MeasurementModel::RangeBearing2D) {
    return;
  }
  const int bidx = (z.model == MeasurementModel::Bearing2D) ? 0 : 1;
  if (z.value.size() <= bidx) return;
  const auto pred = predictMeasurement(z.model, tr_pred.state,
                                       z.sensor_position_enu);
  if (pred.H.rows() <= bidx) return;
  const double beta_pred = pred.z_pred(bidx);
  const double beta_obs  = z.value(bidx);
  const double r = wrapAngle(beta_obs - beta_pred);
  const Eigen::RowVectorXd Hb = pred.H.row(bidx);
  const double state_var =
      (Hb * tr_pred.covariance * Hb.transpose())(0, 0);
  const double R_bb =
      (z.covariance.rows() > bidx && z.covariance.cols() > bidx)
          ? z.covariance(bidx, bidx)
          : 0.0;
  const double S = (state_var + R_bb) / weight;
  const double dx = tr_pred.state(0) - z.sensor_position_enu.x();
  const double dy = tr_pred.state(1) - z.sensor_position_enu.y();
  BearingInnovation obs;
  obs.time = z.time;
  obs.track_id = tr_pred.id;
  obs.innovation_rad = r;
  obs.variance_rad2 = S;
  obs.predicted_state_var_rad2 = state_var / weight;
  obs.range_m = std::hypot(dx, dy);
  sink->onBearingInnovation(obs);
}

// General-purpose innovation emission. The estimator's update used
// (z, R, predicted P); we reproduce ν = z − h(x̂⁻), H = ∂h/∂x at x̂⁻,
// and S = HPHᵀ + R from the same pre-update state so the consumer sees
// exactly the innovation the filter saw. Skips the soft-update path
// (gating soft-update emission to the bearing-only port until a
// β-weighted variant is designed; see IInnovationSink rationale).
void emitInnovation(IInnovationSink* sink, const Track& tr_pred,
                    const Measurement& z) {
  if (sink == nullptr) return;
  const auto pred = predictMeasurement(z.model, tr_pred.state,
                                       z.sensor_position_enu);
  if (pred.z_pred.size() == 0 || pred.H.rows() == 0) return;
  const Eigen::VectorXd nu =
      measurementResidual(z.model, z.value, pred.z_pred);
  // R must exist at the right shape; an adapter that left covariance
  // empty has already failed validation upstream. Bail out on shape
  // mismatch rather than silently injecting zeros.
  if (z.covariance.rows() < nu.size() || z.covariance.cols() < nu.size()) {
    return;
  }
  const Eigen::MatrixXd R = z.covariance.topLeftCorner(nu.size(), nu.size());
  const Eigen::MatrixXd S =
      pred.H * tr_pred.covariance * pred.H.transpose() + R;
  InnovationEvent e;
  e.time = z.time;
  e.track_id = tr_pred.id;
  e.sensor = z.sensor;
  e.source_id = z.source_id;
  e.model = z.model;
  e.residual = nu;
  e.S = S;
  e.R = R;
  e.dim = static_cast<std::size_t>(nu.size());
  sink->onInnovation(e);
}

}  // namespace

Tracker::Tracker(const IEstimator& estimator,
                 const IDataAssociator& associator,
                 TrackManager& manager,
                 double miss_timeout_seconds)
    : estimator_(estimator),
      associator_(associator),
      manager_(manager),
      miss_timeout_seconds_(miss_timeout_seconds) {}

void Tracker::process(const Measurement& z) {
  if (has_high_water_ && z.time < high_water_) {
    if (reject_stale_) {
      ++stale_dropped_;
      return;
    }
  } else {
    high_water_ = z.time;
    has_high_water_ = true;
  }

  manager_.predictAll(estimator_, z.time);

  const std::vector<Measurement> batch{z};
  const AssociationResult result =
      associator_.associate(manager_.tracks(), batch, &estimator_);

  if (!result.matches.empty()) {
    const std::size_t ti = result.matches.front().first;
    Track& tr = manager_.mutableTracks()[ti];
    emitBearingInnovationIfApplicable(bearing_innov_sink_, tr, z);
    emitInnovation(innov_sink_, tr, z);
    estimator_.update(tr, z);
    {
      Track::SourceTouch touch;
      touch.sensor = z.sensor;
      touch.source_id = z.source_id;
      touch.time = z.time;
      if (z.model == MeasurementModel::Position2D && z.value.size() >= 2) {
        touch.value_enu = z.value.head<2>();
        if (z.covariance.rows() >= 2 && z.covariance.cols() >= 2) {
          touch.covariance = z.covariance.topLeftCorner<2, 2>();
        }
      }
      touch.sensor_position_enu = z.sensor_position_enu;
      touch.own_position_std_m = z.sensor_position_std_m;
      touch.covariance_is_default = z.covariance_is_default;
      tr.recent_contributions.push_back(std::move(touch));
    }
    bool has_src = false;
    for (const auto& s : tr.contributing_sources) {
      if (s == z.source_id) {
        has_src = true;
        break;
      }
    }
    if (!has_src) tr.contributing_sources.push_back(z.source_id);
    const TrackId id = tr.id;
    manager_.recordHit(id);
    manager_.noteObservation(id, z.time);
    manager_.recordUpdated(id, z.time);
  } else if (canInitiateTrack(z.model)) {
    Track seed = estimator_.initiate(z);
    manager_.add(seed, z.time);
  }
  // else: a bearing-only measurement that gated to no track has no
  // observable range — drop it rather than seed a garbage position
  // (passive sensors don't initiate; see canInitiateTrack).

  const std::int64_t timeout_ns =
      static_cast<std::int64_t>(miss_timeout_seconds_ * 1e9);
  std::vector<TrackId> stale;
  for (const auto& tr : manager_.tracks()) {
    const std::int64_t age =
        z.time.nanos() - manager_.lastObservation(tr.id).nanos();
    if (age > timeout_ns) stale.push_back(tr.id);
  }
  for (const TrackId id : stale) manager_.recordMiss(id);
}

void Tracker::processBatch(const std::vector<Measurement>& scan) {
  if (scan.empty()) return;
  const Timestamp t = scan.front().time;
  if (has_high_water_ && t < high_water_) {
    if (reject_stale_) {
      stale_dropped_ += scan.size();
      return;
    }
  } else {
    high_water_ = t;
    has_high_water_ = true;
  }

  manager_.predictAll(estimator_, t);

  const AssociationResult result =
      associator_.associate(manager_.tracks(), scan, &estimator_);

  const bool soft = result.betas.size() > 0 && result.beta_0.size() > 0;
  std::vector<bool> meas_used(scan.size(), false);

  if (soft) {
    const int M = static_cast<int>(result.betas.rows());
    const int T = static_cast<int>(result.betas.cols());
    for (int ti = 0; ti < T; ++ti) {
      std::vector<Measurement> gated;
      std::vector<double> betas_vec;
      for (int j = 0; j < M; ++j) {
        const double b = result.betas(j, ti);
        if (b > 0.0) {
          gated.push_back(scan[j]);
          betas_vec.push_back(b);
          meas_used[j] = true;
        }
      }
      if (gated.empty()) continue;
      Eigen::VectorXd betas_eig(betas_vec.size());
      for (std::size_t k = 0; k < betas_vec.size(); ++k)
        betas_eig(k) = betas_vec[k];
      Track& tr = manager_.mutableTracks()[ti];
      for (std::size_t k = 0; k < gated.size(); ++k) {
        emitBearingInnovationIfApplicable(bearing_innov_sink_, tr,
                                          gated[k], betas_vec[k]);
      }
      const PdaContext ctx{result.p_d, result.gate_threshold};
      estimator_.softUpdate(tr, gated, betas_eig, result.beta_0(ti), ctx);
      for (const auto& gz : gated) {
        Track::SourceTouch touch;
        touch.sensor = gz.sensor;
        touch.source_id = gz.source_id;
        touch.time = gz.time;
        if (gz.model == MeasurementModel::Position2D && gz.value.size() >= 2) {
          touch.value_enu = gz.value.head<2>();
          if (gz.covariance.rows() >= 2 && gz.covariance.cols() >= 2) {
            touch.covariance = gz.covariance.topLeftCorner<2, 2>();
          }
        }
        touch.sensor_position_enu = gz.sensor_position_enu;
        touch.own_position_std_m = gz.sensor_position_std_m;
        touch.covariance_is_default = gz.covariance_is_default;
        tr.recent_contributions.push_back(std::move(touch));
      }
      const TrackId id = tr.id;
      manager_.recordHit(id);
      manager_.noteObservation(id, t);
      manager_.recordUpdated(id, t);
    }
  } else {
    for (const auto& m : result.matches) {
      const std::size_t ti = m.first;
      const std::size_t mi = m.second;
      Track& tr = manager_.mutableTracks()[ti];
      emitBearingInnovationIfApplicable(bearing_innov_sink_, tr, scan[mi]);
      emitInnovation(innov_sink_, tr, scan[mi]);
      estimator_.update(tr, scan[mi]);
      {
        const Measurement& z = scan[mi];
        Track::SourceTouch touch;
        touch.sensor = z.sensor;
        touch.source_id = z.source_id;
        touch.time = z.time;
        if (z.model == MeasurementModel::Position2D && z.value.size() >= 2) {
          touch.value_enu = z.value.head<2>();
          if (z.covariance.rows() >= 2 && z.covariance.cols() >= 2) {
            touch.covariance = z.covariance.topLeftCorner<2, 2>();
          }
        }
        touch.sensor_position_enu = z.sensor_position_enu;
        touch.own_position_std_m = z.sensor_position_std_m;
        tr.recent_contributions.push_back(std::move(touch));
      }
      bool has_src = false;
      for (const auto& s : tr.contributing_sources) {
        if (s == scan[mi].source_id) { has_src = true; break; }
      }
      if (!has_src) tr.contributing_sources.push_back(scan[mi].source_id);
      const TrackId id = tr.id;
      manager_.recordHit(id);
      manager_.noteObservation(id, t);
      manager_.recordUpdated(id, t);
      meas_used[mi] = true;
    }
  }

  for (std::size_t j = 0; j < scan.size(); ++j) {
    if (!meas_used[j] && canInitiateTrack(scan[j].model)) {
      Track seed = estimator_.initiate(scan[j]);
      manager_.add(seed, t);
    }
    // Unassociated bearing-only measurements are dropped (no observable
    // range to seed a track); they only ever refine existing tracks.
  }

  const std::int64_t timeout_ns =
      static_cast<std::int64_t>(miss_timeout_seconds_ * 1e9);
  std::vector<TrackId> stale;
  for (const auto& tr : manager_.tracks()) {
    const std::int64_t age =
        t.nanos() - manager_.lastObservation(tr.id).nanos();
    if (age > timeout_ns) stale.push_back(tr.id);
  }
  for (const TrackId id : stale) manager_.recordMiss(id);
}

}  // namespace navtracker
