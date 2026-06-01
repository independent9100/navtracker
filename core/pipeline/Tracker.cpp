#include "core/pipeline/Tracker.hpp"

#include <cstdint>
#include <vector>

#include "core/tracking/TrackManager.hpp"

namespace navtracker {

Tracker::Tracker(const IEstimator& estimator,
                 const IDataAssociator& associator,
                 TrackManager& manager,
                 double miss_timeout_seconds)
    : estimator_(estimator),
      associator_(associator),
      manager_(manager),
      miss_timeout_seconds_(miss_timeout_seconds) {}

void Tracker::process(const Measurement& z) {
  manager_.predictAll(estimator_, z.time);

  const std::vector<Measurement> batch{z};
  const AssociationResult result =
      associator_.associate(manager_.tracks(), batch);

  if (!result.matches.empty()) {
    const std::size_t ti = result.matches.front().first;
    Track& tr = manager_.mutableTracks()[ti];
    estimator_.update(tr, z);
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
  } else {
    Track seed = estimator_.initiate(z);
    manager_.add(seed, z.time);
  }

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
  manager_.predictAll(estimator_, t);

  const AssociationResult result =
      associator_.associate(manager_.tracks(), scan);

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
      estimator_.softUpdate(tr, gated, betas_eig, result.beta_0(ti));
      const TrackId id = tr.id;
      manager_.recordHit(id);
      manager_.noteObservation(id, t);
    }
  } else {
    for (const auto& m : result.matches) {
      const std::size_t ti = m.first;
      const std::size_t mi = m.second;
      Track& tr = manager_.mutableTracks()[ti];
      estimator_.update(tr, scan[mi]);
      bool has_src = false;
      for (const auto& s : tr.contributing_sources) {
        if (s == scan[mi].source_id) { has_src = true; break; }
      }
      if (!has_src) tr.contributing_sources.push_back(scan[mi].source_id);
      const TrackId id = tr.id;
      manager_.recordHit(id);
      manager_.noteObservation(id, t);
      meas_used[mi] = true;
    }
  }

  for (std::size_t j = 0; j < scan.size(); ++j) {
    if (!meas_used[j]) {
      Track seed = estimator_.initiate(scan[j]);
      manager_.add(seed, t);
    }
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
