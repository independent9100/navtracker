#include "core/t2t/T2tFuser.hpp"

#include <algorithm>
#include <utility>

#include "core/geo/AxisRotation.hpp"
#include "core/tracking/DatumShift.hpp"

namespace navtracker::t2t {
namespace {

constexpr char kKeySep = '\x1f';  // unit separator; never appears in ids

}  // namespace

std::string T2tFuser::keyOf(const ExternalTrack& t) {
  return t.source_tracker_id + kKeySep + t.source_track_id;
}

T2tFuser::T2tFuser(T2tConfig cfg, const IFusionRule* rule)
    : cfg_(cfg),
      rule_(rule),
      default_rule_(cfg.ci_omega_iterations),
      motion_(cfg.process_noise_accel_psd_m2_s3),
      associator_(cfg),
      stale_guard_(cfg.reject_stale_reports) {}

void T2tFuser::registerSource(const std::string& source_tracker_id,
                              SourcePedigree pedigree) {
  registered_pedigrees_[source_tracker_id] = std::move(pedigree);
}

SourcePedigree T2tFuser::effectivePedigreeFor(const ExternalTrack& t) const {
  if (t.pedigree.has_value()) return *t.pedigree;
  const auto it = registered_pedigrees_.find(t.source_tracker_id);
  if (it != registered_pedigrees_.end()) return it->second;
  return SourcePedigree{};  // all-Unknown
}

bool T2tFuser::process(ExternalTrack report) {
  std::string reason;
  if (!validateExternalTrack(report, &reason)) {
    ++rejected_;
    return false;
  }
  if (!stale_guard_.accept(report)) return false;  // per-source out-of-order
  applyExternalDefaultsIfEmpty(report);
  const std::string key = keyOf(report);
  const Timestamp t = report.time;
  // A strictly newer report closes the current scan: flush it (one cycle) so
  // the lifecycle advances per scan, not per report.
  if (has_pending_ && t.nanos() > pending_time_.nanos()) flushPending();
  sources_[key] = StoredSource{std::move(report), t};
  reported_keys_.insert(key);
  pending_time_ = has_pending_ && pending_time_.nanos() > t.nanos() ? pending_time_ : t;
  has_pending_ = true;
  return true;
}

void T2tFuser::flush() { flushPending(); }

void T2tFuser::flushPending() {
  if (!has_pending_) return;
  const Timestamp t = pending_time_;
  std::set<std::string> reporters;
  reporters.swap(reported_keys_);
  has_pending_ = false;
  runCycle(t, reporters);
}

void T2tFuser::advanceTo(Timestamp t) {
  flushPending();
  if (!has_time_ || t.nanos() > now_.nanos()) runCycle(t, {});
}

// ---------------------------------------------------------------------------
// Prediction helpers
// ---------------------------------------------------------------------------

T2tFuser::PredictedSource T2tFuser::predictSource(const StoredSource& s,
                                                  Timestamp to) const {
  const ExternalTrack& e = s.latest;
  PredictedSource p;
  p.key = keyOf(e);
  p.source_tracker_id = e.source_tracker_id;
  p.source_track_id = e.source_track_id;
  p.last_report = s.last_report;
  p.has_velocity = e.velocity_valid;
  p.mmsi = e.attributes.mmsi;
  p.source_status = e.source_status;
  p.pedigree = effectivePedigreeFor(e);
  p.pessimistic_default = e.covariance_is_pessimistic_default;

  Eigen::Vector4d x = Eigen::Vector4d::Zero();
  x.head<2>() = e.position_enu;
  Eigen::Matrix4d P = Eigen::Matrix4d::Zero();
  P.topLeftCorner<2, 2>() = e.position_cov;
  if (e.velocity_valid) {
    x.segment<2>(2) = e.velocity_enu;
    P.block<2, 2>(2, 2) = e.velocity_cov;
  } else {
    // Position-only source: assume zero velocity with a wide prior so its
    // position uncertainty grows appropriately between reports.
    const double vv = cfg_.source_unknown_velocity_std_mps *
                      cfg_.source_unknown_velocity_std_mps;
    P.block<2, 2>(2, 2) = Eigen::Matrix2d::Identity() * vv;
  }

  const double dt = to.secondsSince(s.last_report);
  if (dt > 0.0) {
    const Eigen::Matrix4d F = motion_.transitionMatrix(dt);
    const Eigen::Matrix4d Q = motion_.processNoise(dt);
    x = F * x;
    P = F * P * F.transpose() + Q;
  }
  p.state = x;
  p.cov = P;
  return p;
}

void T2tFuser::predictFusedForward(FusedState& f, Timestamp to) const {
  const double dt = to.secondsSince(f.track.last_update);
  if (dt <= 0.0) return;
  const Eigen::MatrixXd F = motion_.transitionMatrix(dt);
  const Eigen::MatrixXd Q = motion_.processNoise(dt);
  f.track.state = F * f.track.state;
  f.track.covariance = F * f.track.covariance * F.transpose() + Q;
  f.track.last_update = to;
}

GateCandidate T2tFuser::gateOfFused(const FusedState& f) const {
  GateCandidate g;
  g.position = f.track.state.head<2>();
  g.covariance = f.track.covariance.topLeftCorner<2, 2>();
  g.mmsi = f.track.attributes.mmsi;
  return g;
}

GateCandidate T2tFuser::gateOfSource(const PredictedSource& p) {
  GateCandidate g;
  g.position = p.state.head<2>();
  g.covariance = p.cov.topLeftCorner<2, 2>();
  g.mmsi = p.mmsi;
  return g;
}

// ---------------------------------------------------------------------------
// CI fusion of a fused track's current contributors
// ---------------------------------------------------------------------------

void T2tFuser::fuseInto(FusedState& f,
                        const std::vector<const PredictedSource*>& contributors) {
  const IFusionRule& rule = rule_ ? *rule_ : default_rule_;

  // Independence verdict over the contributors' pedigrees (drives output; CI
  // ignores it in v1).
  std::vector<const SourcePedigree*> peds;
  peds.reserve(contributors.size());
  for (const auto* c : contributors) peds.push_back(&c->pedigree);
  f.independence = fusedIndependence(peds);

  // Position: sequential CI fold over ALL contributors (canonical order).
  Eigen::VectorXd px = contributors.front()->state.head<2>();
  Eigen::MatrixXd pP = contributors.front()->cov.topLeftCorner<2, 2>();
  for (std::size_t i = 1; i < contributors.size(); ++i) {
    const CiResult r = rule.fuse(px, pP, contributors[i]->state.head<2>(),
                                 contributors[i]->cov.topLeftCorner<2, 2>(),
                                 f.independence);
    px = r.x;
    pP = r.P;
  }

  // Velocity: CI fold over contributors that supply a valid velocity. If none,
  // adopt zero velocity with the wide unknown-velocity prior (so a coasting
  // fused track still inflates position sensibly) and mark it not observed.
  bool vel_observed = false;
  Eigen::VectorXd vx = Eigen::VectorXd::Zero(2);
  Eigen::MatrixXd vP =
      Eigen::MatrixXd::Identity(2, 2) * (cfg_.source_unknown_velocity_std_mps *
                                         cfg_.source_unknown_velocity_std_mps);
  for (const auto* c : contributors) {
    if (!c->has_velocity) continue;
    const Eigen::VectorXd cvx = c->state.segment<2>(2);
    const Eigen::MatrixXd cvP = c->cov.block<2, 2>(2, 2);
    if (!vel_observed) {
      vx = cvx;
      vP = cvP;
      vel_observed = true;
    } else {
      const CiResult r = rule.fuse(vx, vP, cvx, cvP, f.independence);
      vx = r.x;
      vP = r.P;
    }
  }

  // Assemble the 4-D fused state (block-diagonal pos/vel; cross-covariance
  // dropped — a documented approximation, algorithm doc §4).
  Eigen::VectorXd state(4);
  state.head<2>() = px;
  state.segment<2>(2) = vx;
  Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(4, 4);
  cov.topLeftCorner<2, 2>() = pP;
  cov.block<2, 2>(2, 2) = vP;
  f.track.state = state;
  f.track.covariance = cov;
  f.track.last_update = now_;
  f.track.velocity_observed = vel_observed;

  // Provenance + attributes.
  f.track.contributing_sources.clear();
  f.contributors.clear();
  std::optional<std::uint32_t> mmsi;
  bool mmsi_conflict = false;
  f.pessimistic_default = false;
  for (const auto* c : contributors) {
    if (std::find(f.track.contributing_sources.begin(),
                  f.track.contributing_sources.end(),
                  c->source_tracker_id) == f.track.contributing_sources.end())
      f.track.contributing_sources.push_back(c->source_tracker_id);
    f.contributors.push_back(
        ContributingTracker{c->source_tracker_id, c->source_track_id, c->last_report});
    if (c->mmsi.has_value()) {
      if (!mmsi.has_value())
        mmsi = c->mmsi;
      else if (*mmsi != *c->mmsi)
        mmsi_conflict = true;
    }
    f.pessimistic_default = f.pessimistic_default || c->pessimistic_default;
  }
  // Adopt an MMSI only if the contributors agree (conflict => assert none;
  // kinematics already won the association, invariant 5).
  f.track.attributes.mmsi = mmsi_conflict ? std::nullopt : mmsi;
  f.last_contrib = now_;
}

// ---------------------------------------------------------------------------
// The fusion cycle
// ---------------------------------------------------------------------------

void T2tFuser::runCycle(Timestamp t, const std::set<std::string>& reporters) {
  now_ = (has_time_ && now_.nanos() > t.nanos()) ? now_ : t;
  has_time_ = true;
  const auto reportedThisScan = [&](const std::string& key) {
    return reporters.count(key) != 0;
  };

  // 1. Predict fresh source tracks to now_; drop long-dead ones (and their
  //    stale birth candidacy) from the store.
  std::vector<PredictedSource> preds;
  std::map<std::string, std::size_t> pred_index;  // key -> index in preds
  for (auto it = sources_.begin(); it != sources_.end();) {
    const double age = now_.secondsSince(it->second.last_report);
    if (age > cfg_.fused_delete_age_s) {
      birth_window_.erase(it->first);  // prune birth candidacy with the source
      it = sources_.erase(it);
      continue;
    }
    if (age <= cfg_.max_report_age_s) {
      pred_index[it->first] = preds.size();
      preds.push_back(predictSource(it->second, now_));
    }
    ++it;
  }

  // 2. Predict all fused tracks forward to now_.
  for (auto& f : fused_) predictFusedForward(f, now_);

  // 3a. Sticky pass: FORMED pairings bypass global assignment while in-gate.
  std::vector<bool> source_used(preds.size(), false);
  std::vector<std::vector<const PredictedSource*>> contributors(fused_.size());
  for (std::size_t fi = 0; fi < fused_.size(); ++fi) {
    for (const auto& [key, pairing] : fused_[fi].pairings) {
      if (!pairing.formed) continue;
      const auto pit = pred_index.find(key);
      if (pit == pred_index.end()) continue;  // source not fresh this cycle
      const std::size_t si = pit->second;
      if (source_used[si]) continue;
      if (associator_.gateDistanceSq(gateOfFused(fused_[fi]),
                                     gateOfSource(preds[si])) <=
          cfg_.gate_chi2_position) {
        source_used[si] = true;
        contributors[fi].push_back(&preds[si]);
      }
    }
  }

  // 3b. Global pass: unpaired fused tracks x unused sources via the associator.
  std::vector<GateCandidate> fused_gates;
  fused_gates.reserve(fused_.size());
  for (const auto& f : fused_) fused_gates.push_back(gateOfFused(f));
  std::vector<GateCandidate> free_source_gates;
  std::vector<std::size_t> free_source_idx;
  for (std::size_t si = 0; si < preds.size(); ++si) {
    if (source_used[si]) continue;
    free_source_gates.push_back(gateOfSource(preds[si]));
    free_source_idx.push_back(si);
  }
  const T2tAssignment assign = associator_.associate(fused_gates, free_source_gates);
  for (const auto& [fi, local_sj] : assign.matches) {
    const std::size_t si = free_source_idx[local_sj];
    source_used[si] = true;
    contributors[fi].push_back(&preds[si]);
  }

  // 4. Birth: only sources that REPORTED this scan and remain unassociated
  //    accrue birth M-of-N (evidence counts the source's own reports, not
  //    ambient cycles driven by other sources' traffic).
  std::vector<std::size_t> births;
  for (const std::size_t si : free_source_idx) {
    if (source_used[si]) continue;                    // matched above
    if (!reportedThisScan(preds[si].key)) continue;   // reporter-driven M-of-N
    auto& win = birth_window_[preds[si].key];
    win.push_back(true);
    while (static_cast<int>(win.size()) > cfg_.fused_confirm_n) win.pop_front();
    const int hits = static_cast<int>(std::count(win.begin(), win.end(), true));
    if (hits >= cfg_.fused_confirm_m) births.push_back(si);
  }
  // Cluster co-located birth candidates so two source trackers reporting the
  // SAME new object in the SAME scan seed ONE fused track (with both as
  // founders), not two duplicates. (Pre-existing tracks were already offered in
  // the global pass; this only groups simultaneous newborns among themselves.)
  std::vector<std::vector<std::size_t>> clusters;
  for (const std::size_t si : births) {
    bool joined = false;
    for (auto& members : clusters) {
      if (associator_.gateDistanceSq(gateOfSource(preds[members.front()]),
                                     gateOfSource(preds[si])) <=
          cfg_.gate_chi2_position) {
        members.push_back(si);
        joined = true;
        break;
      }
    }
    if (!joined) clusters.push_back({si});
  }
  for (const auto& members : clusters) {
    FusedState f;
    f.track.id = TrackId{next_fused_id_++};  // monotonic, never reused
    f.track.status = TrackStatus::Tentative;
    f.track.state = Eigen::VectorXd::Zero(4);
    f.track.covariance = Eigen::MatrixXd::Identity(4, 4);
    f.track.last_update = now_;
    Pairing founding;
    founding.hits = cfg_.pair_confirm_hits;
    founding.formed = true;  // founding sources are sticky from birth
    for (const std::size_t si : members) f.pairings[preds[si].key] = founding;
    fused_.push_back(std::move(f));
    contributors.emplace_back();
    for (const std::size_t si : members) {
      contributors.back().push_back(&preds[si]);
      source_used[si] = true;
      birth_window_.erase(preds[si].key);
    }
    if (sink_)
      sink_->onFusedTrackInitiated(
          {fused_.back().track.id, now_, TrackStatus::Tentative});
  }
  // Reset birth windows for sources that got associated (not birthing).
  for (std::size_t si = 0; si < preds.size(); ++si)
    if (source_used[si]) birth_window_.erase(preds[si].key);

  // 5. Per fused track: pairing hysteresis, fuse, confirm, coast.
  for (std::size_t fi = 0; fi < fused_.size(); ++fi) {
    FusedState& f = fused_[fi];

    // Which contributing sources reported THIS scan (vs coasted-fresh silent).
    std::set<std::string> contributed_keys;
    bool reporter_contributed = false;
    for (const auto* c : contributors[fi]) {
      contributed_keys.insert(c->key);
      if (reportedThisScan(c->key)) reporter_contributed = true;
    }

    // Pairing bookkeeping: hit when the paired source reported this scan AND
    // gated in; miss when it reported out-of-gate OR has gone stale (dropped
    // from preds); no change while it is fresh but silent between its own
    // reports (so a slow source's pairing survives its reporting cadence).
    for (auto pit = f.pairings.begin(); pit != f.pairings.end();) {
      const std::string& key = pit->first;
      const bool contributed = contributed_keys.count(key) != 0;
      const bool reported = reportedThisScan(key);
      const bool stale = pred_index.find(key) == pred_index.end();
      if (contributed && reported) {
        pit->second.hits += 1;
        pit->second.misses = 0;
        if (pit->second.hits >= cfg_.pair_confirm_hits) pit->second.formed = true;
        ++pit;
      } else if ((reported && !contributed) || stale) {
        pit->second.misses += 1;
        if (pit->second.misses >= cfg_.pair_break_misses)
          pit = f.pairings.erase(pit);
        else
          ++pit;
      } else {
        ++pit;  // fresh but silent this scan: pairing holds unchanged
      }
    }
    for (const auto* c : contributors[fi])
      if (f.pairings.find(c->key) == f.pairings.end()) {
        Pairing pr;
        pr.hits = 1;
        f.pairings[c->key] = pr;
      }

    if (!contributors[fi].empty()) {
      // confirm_window is empty only before the first fuse -> the birth scan.
      const bool was_new = f.confirm_window.empty();
      // Canonical contributor order (by source key) so the sequential CI fold
      // is deterministic and independent of association-pass order.
      std::sort(contributors[fi].begin(), contributors[fi].end(),
                [](const PredictedSource* a, const PredictedSource* b) {
                  return a->key < b->key;
                });
      fuseInto(f, contributors[fi]);
      // Confirm M-of-N counts scans with a FRESH (reporter) contribution — a
      // memoryless re-fuse from purely coasted sources is not fresh evidence.
      f.confirm_window.push_back(reporter_contributed);
      while (static_cast<int>(f.confirm_window.size()) > cfg_.fused_confirm_n)
        f.confirm_window.pop_front();

      if (f.track.status != TrackStatus::Confirmed) {
        bool confirm = false;
        if (cfg_.trust_source_status)
          for (const auto* c : contributors[fi])
            if (reportedThisScan(c->key) && c->source_status == TrackStatus::Confirmed)
              confirm = true;
        if (!confirm &&
            static_cast<int>(std::count(f.confirm_window.begin(),
                                        f.confirm_window.end(), true)) >=
                cfg_.fused_confirm_m)
          confirm = true;
        if (confirm) f.track.status = TrackStatus::Confirmed;
      }

      // Initiated fired at birth; fire Updated only when a fresh report
      // actually updated the estimate (not on a purely-coasted re-fuse).
      if (!was_new && reporter_contributed && sink_)
        sink_->onFusedTrackUpdated({f.track.id, now_, f.track.status});
      if (f.track.status == TrackStatus::Confirmed && !f.confirmed_fired) {
        f.confirmed_fired = true;
        if (sink_) sink_->onFusedTrackConfirmed({f.track.id, now_, f.track.status});
      }
    } else {
      // Full coast: no fresh contributor at all. Nothing is currently fused, so
      // every "current-fusion" field must stop reporting stale corroboration —
      // the ContributingTracker list, the source-id list inherited by
      // TrackOutput, the independence verdict, and the pessimistic-default flag.
      // (Track identity, status, kinematics, and attributes persist.)
      if (f.track.status != TrackStatus::Deleted)
        f.track.status = TrackStatus::Coasting;
      f.contributors.clear();
      f.track.contributing_sources.clear();
      f.independence = IndependenceClass::SingleSource;
      f.pessimistic_default = false;
      f.confirm_window.push_back(false);
      while (static_cast<int>(f.confirm_window.size()) > cfg_.fused_confirm_n)
        f.confirm_window.pop_front();
    }
  }

  // 6. Delete fused tracks with no fresh contributor for too long.
  std::vector<FusedState> survivors;
  survivors.reserve(fused_.size());
  for (auto& f : fused_) {
    const double idle = now_.secondsSince(f.last_contrib);
    if (idle > cfg_.fused_delete_age_s) {
      if (sink_)
        sink_->onFusedTrackDeleted({f.track.id, now_, f.track.status});
      continue;  // fire-before-erase, then drop
    }
    survivors.push_back(std::move(f));
  }
  fused_ = std::move(survivors);
}

// ---------------------------------------------------------------------------
// Output
// ---------------------------------------------------------------------------

std::vector<FusedTrackOutput> T2tFuser::fusedTracks() const {
  std::vector<FusedTrackOutput> out;
  if (!datum_.has_value()) return out;  // no frame to convert into yet
  out.reserve(fused_.size());
  for (const auto& f : fused_) {
    out.push_back(toFusedTrackOutput(f.track, *datum_, f.contributors,
                                     f.independence, f.pessimistic_default));
  }
  return out;
}

std::vector<T2tFuser::FusedEnuState> T2tFuser::fusedTracksEnu() const {
  std::vector<FusedEnuState> out;
  out.reserve(fused_.size());
  for (const auto& f : fused_) {
    FusedEnuState s;
    s.id = f.track.id;
    s.status = f.track.status;
    s.position = f.track.state.head<2>();
    s.velocity = f.track.state.segment<2>(2);
    s.position_cov = f.track.covariance.topLeftCorner<2, 2>();
    out.push_back(s);
  }
  return out;
}

// ---------------------------------------------------------------------------
// Datum recenter: re-express cached ENU state into the new frame.
// ---------------------------------------------------------------------------

void T2tFuser::onDatumRecentered(const geo::Datum& old_d, const geo::Datum& new_d) {
  Eigen::MatrixXd no_imm_means;             // empty
  std::vector<Eigen::MatrixXd> no_imm_cov;  // empty
  for (auto& f : fused_) {
    shiftStateOnDatumChange(f.track.state, f.track.covariance, no_imm_means,
                            no_imm_cov, old_d, new_d);
  }
  // Stored source reports (still in the old frame until their next report).
  const Eigen::Matrix2d R = geo::datumAxisRotation(old_d, new_d);
  for (auto& [key, s] : sources_) {
    (void)key;
    ExternalTrack& e = s.latest;
    const auto g = old_d.toGeodetic(Eigen::Vector3d(e.position_enu.x(),
                                                    e.position_enu.y(), 0.0));
    const Eigen::Vector3d enu = new_d.toEnu(g);
    e.position_enu = Eigen::Vector2d(enu.x(), enu.y());
    e.position_cov = R * e.position_cov * R.transpose();
    if (e.velocity_valid) {
      e.velocity_enu = R * e.velocity_enu;
      e.velocity_cov = R * e.velocity_cov * R.transpose();
    }
  }
  datum_ = new_d;
}

}  // namespace navtracker::t2t
