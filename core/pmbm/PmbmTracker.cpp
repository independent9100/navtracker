#include "core/pmbm/PmbmTracker.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <tuple>
#include <utility>

#include <Eigen/Dense>

#include "core/association/Murty.hpp"
#include "core/estimation/MeasurementModels.hpp"
#include "core/pipeline/BiasCorrection.hpp"
#include "core/pipeline/SourceTouchPopulate.hpp"

namespace navtracker::pmbm {

namespace {
// Build the constant-velocity transition matrix F for state layout
// (px, py, vx, vy [, ω]) over time interval dt. Identity on velocity
// (and ω for 5-state); position += dt · velocity. Exact for CV
// motion; approximate for CT (the ω-coupled position update is lost,
// but autoferry tracks spend most time straight, so the
// approximation is small).
Eigen::MatrixXd cvTransitionMatrix(int n_state, double dt) {
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(n_state, n_state);
  if (n_state >= 4) {
    F(0, 2) = dt;
    F(1, 3) = dt;
  }
  return F;
}
}  // namespace

void rtsSmoothTrajectory(std::vector<TrajectoryPoint>& trajectory) {
  const std::size_t T = trajectory.size();
  if (T < 2) return;
  // Walk backward k = T-2 .. 0. Smoothed at k uses smoothed at k+1.
  // F derived from dt assuming CV motion layout (px, py, vx, vy[, ω]).
  // Exact for CV2D / CV5; approximate for CT (ω-coupled position
  // update lost). Phase 6 iter 2 measured F=I to regress anchored
  // (tight-AIS) by copying the end position back through history;
  // CV-F restores the position-velocity coupling that anchors the
  // smoother to actual motion.
  for (std::size_t k_plus = T - 1; k_plus-- > 0;) {
    auto& curr = trajectory[k_plus];          // x_filt_k
    const auto& next = trajectory[k_plus + 1];  // smoothed at k+1
    if (curr.covariance.rows() == 0 ||
        next.predicted_covariance.rows() == 0) continue;
    const double dt = next.time.seconds() - curr.time.seconds();
    const int n = static_cast<int>(curr.state.size());
    const Eigen::MatrixXd F = cvTransitionMatrix(n, dt);
    // G_k = P_filt_k · F^T · P_pred_{k+1}^{-1}, computed via LDLT
    // instead of naive .inverse() — the predicted covariance can be
    // near-singular on long trajectory windows (Phase 8 R2 fix).
    Eigen::LDLT<Eigen::MatrixXd> ldlt(next.predicted_covariance);
    if (ldlt.info() != Eigen::Success) continue;
    const Eigen::VectorXd diag = ldlt.vectorD();
    if ((diag.array() <= 0.0).any()) continue;
    // G^T = P_pred^{-1} · (P_filt · F^T)^T = P_pred^{-1} · F · P_filt
    const Eigen::MatrixXd PF = curr.covariance * F.transpose();
    const Eigen::MatrixXd G = ldlt.solve(PF.transpose()).transpose();
    // x_smooth_k = x_filt_k + G · (x_smooth_{k+1} − x_pred_{k+1})
    curr.state = curr.state +
                 G * (next.state - next.predicted_state);
    // P_smooth_k = P_filt_k + G · (P_smooth_{k+1} − P_pred_{k+1}) · G^T
    Eigen::MatrixXd P_sm = curr.covariance +
        G * (next.covariance - next.predicted_covariance) * G.transpose();
    // Resymmetrise to defend against accumulated rounding asymmetry
    // (Phase 8 R2 fix). LDLT factorisation upstream requires symmetric
    // input; without this the next iteration silently drifts.
    curr.covariance = 0.5 * (P_sm + P_sm.transpose());
  }
}

void rebuildPerTrackViewFromFlat(PmbmDensity& density) {
  density.tracks.clear();
  density.tracked_mbm.clear();
  if (density.mbm.empty()) return;

  std::map<BernoulliId, std::size_t> id_to_track_idx;
  for (const auto& gh : density.mbm) {
    for (const auto& b : gh.bernoullis) {
      auto it = id_to_track_idx.find(b.id);
      if (it == id_to_track_idx.end()) {
        const std::size_t idx = density.tracks.size();
        id_to_track_idx.emplace(b.id, idx);
        PmbmTrack t;
        t.id = b.id;
        t.birth_time = b.last_update;
        density.tracks.push_back(std::move(t));
      } else {
        auto& t = density.tracks[it->second];
        if (b.last_update.seconds() < t.birth_time.seconds()) {
          t.birth_time = b.last_update;
        }
      }
    }
  }

  density.tracked_mbm.resize(density.mbm.size());
  for (std::size_t p = 0; p < density.mbm.size(); ++p) {
    const auto& gh = density.mbm[p];
    auto& tg = density.tracked_mbm[p];
    tg.weight = gh.weight;
    tg.log_weight = gh.log_weight;
    tg.hyp_index.assign(density.tracks.size(),
                        TrackedGlobalHypothesis::kAbsent);
    for (const auto& b : gh.bernoullis) {
      const std::size_t i = id_to_track_idx.at(b.id);
      auto& track = density.tracks[i];
      TrackHypothesis th;
      th.existence_probability = b.existence_probability;
      th.mean = b.mean;
      th.covariance = b.covariance;
      th.imm_means = b.imm_means;
      th.imm_covariances = b.imm_covariances;
      th.imm_mode_probabilities = b.imm_mode_probabilities;
      th.last_update = b.last_update;
      th.trajectory = b.trajectory;
      tg.hyp_index[i] = static_cast<int>(track.hypotheses.size());
      track.hypotheses.push_back(std::move(th));
    }
  }
}

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

// TPMBM trajectory append + window trim. Called after detection
// (post-update state) and after misdetection (post-predict state).
// `predicted_mean` / `predicted_cov` are the PRE-update state at
// this scan (x_{k|k-1}, P_{k|k-1}) — needed by Phase 4(C) RTS
// smoothing. For births and misdetections we pass the post-predict
// state for both pred and filt, so the smoother's first step is a
// no-op at those points (G ≈ I, smoothed = filtered).
// Skips when window=0 (TPMBM disabled).
void appendTrajectoryPoint(Bernoulli& b, std::size_t window_scans,
                           Timestamp t,
                           const Eigen::VectorXd& predicted_mean,
                           const Eigen::MatrixXd& predicted_cov) {
  if (window_scans == 0) return;
  TrajectoryPoint p;
  p.time = t;
  p.state = b.mean;
  p.covariance = b.covariance;
  p.predicted_state = predicted_mean;
  p.predicted_covariance = predicted_cov;
  b.trajectory.push_back(std::move(p));
  if (b.trajectory.size() > window_scans) {
    const std::size_t drop = b.trajectory.size() - window_scans;
    b.trajectory.erase(b.trajectory.begin(),
                       b.trajectory.begin() + drop);
  }
}

// Numerically stable log(Σ exp(x_i)). Returns -inf for empty input.
double logSumExp(const std::vector<double>& v) {
  if (v.empty()) return -kInf;
  double m = -kInf;
  for (double x : v)
    if (x > m) m = x;
  if (m == -kInf) return -kInf;
  double s = 0.0;
  for (double x : v) s += std::exp(x - m);
  return m + std::log(s);
}

}  // namespace

PmbmTracker::PmbmTracker(const IEstimator& estimator, Config cfg,
                         BirthModelFn birth_model)
    : estimator_(estimator),
      cfg_(cfg),
      birth_model_(std::move(birth_model)) {
  // Fail-loud (R9): source_aware_misdetection and use_sensor_activity are
  // ALTERNATIVE miss models and MUST NOT be combined. With both on, the identity
  // gate marks an empty scan "not observable" and short-circuits BEFORE the
  // activity model, silently blocking the cooperative stale-timeout retirement —
  // a never-retiring ghost track. Refuse the combination rather than document it
  // (the R8.8 lesson: don't let bad config misbehave quietly).
  if (cfg_.source_aware_misdetection && cfg_.use_sensor_activity) {
    throw std::invalid_argument(
        "PmbmTracker: source_aware_misdetection and use_sensor_activity are "
        "alternative miss models and cannot be combined — the identity gate "
        "short-circuits the activity model on empty scans, silently blocking the "
        "cooperative stale-timeout retirement (never-retiring ghost track). Use "
        "use_sensor_activity alone (the honest per-duty-cycle coverage model). See "
        "R9 in docs/superpowers/plans/2026-07-02-static-branch-review-fixes.md.");
  }
}

void PmbmTracker::predict(Timestamp to) {
  if (!has_current_time_) {
    current_time_ = to;
    has_current_time_ = true;
    if (birth_model_) {
      auto births = birth_model_(to, 0.0);
      density_.ppp.insert(density_.ppp.end(), births.begin(), births.end());
    }
    return;
  }

  const double dt = to.secondsSince(current_time_);
  if (dt <= 0.0) {
    current_time_ = to;
    return;
  }

  for (auto& c : density_.ppp) {
    c.weight *= cfg_.survival_probability;
    Track t = toTrack(c, current_time_);
    estimator_.predict(t, to);
    fromTrack(c, t);
  }

  for (auto& h : density_.mbm) {
    for (auto& b : h.bernoullis) {
      b.existence_probability *= cfg_.survival_probability;
      Track t = toTrack(b);
      estimator_.predict(t, to);
      fromTrack(b, t);
    }
  }

  if (birth_model_) {
    auto births = birth_model_(to, dt);
    density_.ppp.insert(density_.ppp.end(), births.begin(), births.end());
  }

  current_time_ = to;
}

// Build one Gaussian "new-target Bernoulli" per measurement by folding
// the current PPP through the measurement likelihood (§3.2 of
// docs/algorithms/pmbm-design.md).
//
// For each measurement z_l:
//   ρ_target_l = Σ_i p_D · w_i^u · ℓ(z_l | c_i)
//   ρ_total_l  = ρ_target_l + λ_FA(z_l)
//   r_new_l    = ρ_target_l / ρ_total_l            (existence | detection)
//   f_new_l(x) = Σ_i p_D · w_i^u · ℓ(z_l | c_i) · g(x | c_i, z_l) / ρ_target_l
//
// Phase 1 collapses the posterior mixture {g(x|c_i, z_l)} to a
// moment-matched single Gaussian. Tractable, standard, and what the
// García-Fernández/Williams Phase 1 baseline does.
std::vector<PmbmTracker::NewTargetCandidate>
PmbmTracker::buildNewTargetCandidates(
    const std::vector<Measurement>& scan) const {
  std::vector<NewTargetCandidate> out;
  out.reserve(scan.size());

  for (const auto& z : scan) {
    NewTargetCandidate cand;
    // W5.2: the PPP posterior is moment-matched at current_time_ (== t_max).
    cand.state_time = current_time_;

    // Per-sensor (P_D, λ_C) when a detection model is wired; Config
    // fallback otherwise. Matches MhtTracker's per-measurement
    // paramsFor lookup so PMBM is on the same footing for the bench A/B.
    const double pD_z = detection_model_
        ? detection_model_->paramsFor(z).probability_of_detection
        : cfg_.probability_of_detection;
    const double lambda_z = detection_model_
        ? detection_model_->paramsFor(z).clutter_intensity
        : cfg_.clutter_intensity;

    if (density_.ppp.empty()) {
      cand.rho_target = 0.0;
      cand.rho_total = lambda_z;
      cand.rho_target_unsuppressed = 0.0;
      cand.rho_total_unsuppressed = lambda_z;
      out.push_back(std::move(cand));
      continue;
    }

    std::vector<double> log_weights;
    std::vector<Track> updated_tracks;
    log_weights.reserve(density_.ppp.size());
    updated_tracks.reserve(density_.ppp.size());

    const double log_pD = std::log(pD_z);

    for (const auto& c : density_.ppp) {
      if (c.weight <= 0.0) continue;
      Track t = toTrack(c, current_time_);
      const double log_lik = estimator_.logLikelihood(t, z);
      Track t_upd = t;
      estimator_.update(t_upd, z);
      log_weights.push_back(std::log(c.weight) + log_pD + log_lik);
      updated_tracks.push_back(std::move(t_upd));
    }

    const double log_rho_target = logSumExp(log_weights);
    cand.rho_target = std::exp(log_rho_target);
    cand.rho_total = cand.rho_target + lambda_z;

    if (cand.rho_target > 0.0 && !updated_tracks.empty()) {
      // Moment-match the posterior mixture to a single Gaussian.
      const auto n_state = updated_tracks.front().state.size();
      Eigen::VectorXd mean = Eigen::VectorXd::Zero(n_state);
      double wsum = 0.0;
      for (std::size_t i = 0; i < updated_tracks.size(); ++i) {
        const double w = std::exp(log_weights[i] - log_rho_target);
        mean += w * updated_tracks[i].state;
        wsum += w;
      }
      if (wsum > 0.0) mean /= wsum;
      Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(n_state, n_state);
      for (std::size_t i = 0; i < updated_tracks.size(); ++i) {
        const double w = std::exp(log_weights[i] - log_rho_target);
        const Eigen::VectorXd d = updated_tracks[i].state - mean;
        cov += w * (updated_tracks[i].covariance + d * d.transpose());
      }
      if (wsum > 0.0) cov /= wsum;
      cand.mean = std::move(mean);
      cand.covariance = std::move(cov);

      // Per-mode moment-matched IMM mixture across all post-update
      // components (Phase 2). Each component has the same mode count
      // K (an IMM invariant); the merged mixture's per-mode mean and
      // covariance are weighted by w_i · μ_i[j], with merged mode
      // probabilities μ[j] = (Σ_i w_i · μ_i[j]) / Σ_i w_i.
      //
      // Drops components whose imm_* are empty (single-Gaussian PPP)
      // — they contributed via the moment-matched (mean, cov) above
      // but can't inform the IMM ensemble. The IMM-only-PPP common
      // case (every PPP component carries IMM, via the measurement-
      // driven birth path that calls estimator.initiate) sees every
      // component contribute, with the dominant-component case as the
      // single-contributor degenerate.
      int K = 0;
      int n_imm_state = 0;
      for (const auto& t : updated_tracks) {
        if (t.imm_means.cols() > 0) {
          K = static_cast<int>(t.imm_means.cols());
          n_imm_state = static_cast<int>(t.imm_means.rows());
          break;
        }
      }
      if (K > 0) {
        Eigen::VectorXd modep_acc = Eigen::VectorXd::Zero(K);
        std::vector<Eigen::VectorXd> mode_mean_acc(K, Eigen::VectorXd::Zero(n_imm_state));
        std::vector<double> mode_w_acc(K, 0.0);
        double wsum_imm = 0.0;
        for (std::size_t i = 0; i < updated_tracks.size(); ++i) {
          const auto& t = updated_tracks[i];
          if (t.imm_means.cols() != K) continue;
          const double w = std::exp(log_weights[i] - log_rho_target);
          wsum_imm += w;
          for (int j = 0; j < K; ++j) {
            const double mw = w * t.imm_mode_probabilities(j);
            modep_acc(j) += mw;
            mode_mean_acc[j] += mw * t.imm_means.col(j);
            mode_w_acc[j] += mw;
          }
        }
        if (wsum_imm > 0.0) {
          Eigen::MatrixXd merged_means(n_imm_state, K);
          std::vector<Eigen::MatrixXd> merged_covs(
              K, Eigen::MatrixXd::Zero(n_imm_state, n_imm_state));
          Eigen::VectorXd merged_modep = modep_acc / wsum_imm;
          for (int j = 0; j < K; ++j) {
            if (mode_w_acc[j] > 0.0) {
              merged_means.col(j) = mode_mean_acc[j] / mode_w_acc[j];
            } else {
              merged_means.col(j).setZero();
            }
          }
          for (std::size_t i = 0; i < updated_tracks.size(); ++i) {
            const auto& t = updated_tracks[i];
            if (t.imm_means.cols() != K) continue;
            const double w = std::exp(log_weights[i] - log_rho_target);
            for (int j = 0; j < K; ++j) {
              const double mw = w * t.imm_mode_probabilities(j);
              if (mw <= 0.0) continue;
              const Eigen::VectorXd d =
                  t.imm_means.col(j) - merged_means.col(j);
              merged_covs[j] +=
                  mw * (t.imm_covariances[j] + d * d.transpose());
            }
          }
          for (int j = 0; j < K; ++j) {
            if (mode_w_acc[j] > 0.0) merged_covs[j] /= mode_w_acc[j];
          }
          cand.imm_means = std::move(merged_means);
          cand.imm_covariances = std::move(merged_covs);
          cand.imm_mode_probabilities = std::move(merged_modep);
        }
      }
    } else {
      // No information from PPP; leave mean/cov empty — the candidate
      // contributes only clutter mass and would never be picked by an
      // assignment that produces a real Bernoulli (existence ≈ 0).
      cand.mean = Eigen::VectorXd::Zero(0);
      cand.covariance = Eigen::MatrixXd::Zero(0, 0);
    }

    // Task A + Stage 1 static-obstacle: scale rho_target by the combined birth
    // prior at the birth position (cand.mean, moment-matched from the PPP
    // posterior above), and compute the R1 phantom-birth gate reference. One
    // model evaluation (birthPriorsAt) shared with applyBirthPriors — see the
    // helper for the obstacle-relaxed / land-kept semantics. Empty cand.mean
    // (size<2) → priors are all 0 → no-op (rho_target is already 0 there).
    {
      const SuppressedBirth sb =
          applyBirthPriors(cand.rho_target, lambda_z, birthPriorsAt(cand.mean));
      cand.rho_target = sb.rho_target;
      cand.rho_total = sb.rho_total;
      cand.rho_target_unsuppressed = sb.rho_target_unsuppressed;
      cand.rho_total_unsuppressed = sb.rho_total_unsuppressed;
    }

    out.push_back(std::move(cand));
  }

  return out;
}

// Reuter 2014 Adaptive Birth: one candidate per measurement, decoupled
// from the PPP intensity. Spatial state comes from estimator.initiate
// (mean at z, covariance from the birth Q/R); existence prior is the
// configured λ_birth, balanced against per-sensor λ_C. See
// PmbmTracker::Config::adaptive_birth for math/rationale.
std::vector<PmbmTracker::NewTargetCandidate>
PmbmTracker::buildAdaptiveBirthCandidates(
    const std::vector<Measurement>& scan) const {
  std::vector<NewTargetCandidate> out;
  out.reserve(scan.size());

  for (const auto& z : scan) {
    NewTargetCandidate cand;
    // W5.2: the adaptive-birth state comes from initiate(z) — un-propagated, so
    // it lives at the measurement's own time, not t_max.
    cand.state_time = z.time;

    const double lambda_z = detection_model_
        ? detection_model_->paramsFor(z).clutter_intensity
        : cfg_.clutter_intensity;

    if (!canInitiateTrack(z.model) || !isMeasurementCovariancePsd(z.covariance)) {
      // Bearing-only, otherwise non-initiable, or malformed (empty/non-PSD
      // covariance) measurement: never births a Bernoulli. The total
      // intensity is still the clutter mass so assignment cells stay balanced.
      cand.rho_target = 0.0;
      cand.rho_total = lambda_z;
      cand.rho_target_unsuppressed = 0.0;
      cand.rho_total_unsuppressed = lambda_z;
      out.push_back(std::move(cand));
      continue;
    }

    // Smart-birth gate, ported from the legacy measurement-driven
    // path. Under K=1 the assignment is greedy per measurement-column
    // and a new-target row whose log-cost exceeds the existing
    // Bernoulli's update cost will id-flap. Suppressing the candidate
    // for a measurement already explained by a high-r Bernoulli keeps
    // the existing track stable. The Reuter formulation is correct
    // under exact (K large) enumeration; the gate is the standard
    // workaround for K=1 deployments.
    if (cfg_.smart_birth_skip_existing) {
      bool claimed = false;
      for (const auto& h : density_.mbm) {
        for (const auto& b : h.bernoullis) {
          if (b.existence_probability < cfg_.smart_birth_skip_r_min) continue;
          Track tb = toTrack(b);
          if (estimator_.gate(tb, z, cfg_.smart_birth_skip_gate)) {
            claimed = true;
            break;
          }
        }
        if (claimed) break;
      }
      if (claimed) {
        cand.rho_target = 0.0;
        cand.rho_total = lambda_z;
        cand.rho_target_unsuppressed = 0.0;
        cand.rho_total_unsuppressed = lambda_z;
        out.push_back(std::move(cand));
        continue;
      }
    }

    Track t = estimator_.initiate(z);
    cand.mean = t.state;
    cand.covariance = t.covariance;
    cand.imm_means = t.imm_means;
    cand.imm_covariances = t.imm_covariances;
    cand.imm_mode_probabilities = t.imm_mode_probabilities;

    // Per-sensor λ_birth lookup (Phase 9 review Finding 6B): mirror the
    // ISensorDetectionModel per-(sensor, source) λ_C plumbing. Empty
    // map → scalar fallback (bit-identical to legacy).
    // Task 1 (2026-06-24): when birth_existence_target > 0, derive
    // λ_birth from λ_C so r_new = target exactly, independent of λ_C.
    //   λ_birth = (r*/(1−r*))·λ_C  ⇒  r_new = λ_birth/(λ_birth+λ_C) = r*.
    // This is clutter-invariant: autoferry λ_C=1e-4, philos λ_C=2.7e-6,
    // AIS λ_C=1e-9 all yield r_new = target without manual per-sensor
    // tuning. Legacy path unchanged when target = 0.
    double lambda_birth = cfg_.lambda_birth;
    if (cfg_.birth_existence_target > 0.0 && cfg_.birth_existence_target < 1.0) {
      // Clutter-invariant: choose λ_birth so r_new == target for this z.
      // Guard: r must be in (0, 1); values >= 1.0 would divide by zero
      // or go negative. Fall through to the lambda_birth_per_sensor /
      // scalar path instead (finite, predictable).
      const double r = cfg_.birth_existence_target;
      lambda_birth = (r / (1.0 - r)) * lambda_z;
    } else if (!cfg_.lambda_birth_per_sensor.empty()) {
      auto it = cfg_.lambda_birth_per_sensor.find(z.sensor);
      if (it != cfg_.lambda_birth_per_sensor.end()) {
        lambda_birth = it->second;
      }
    }
    // Task A + Stage 1 static-obstacle: scale the birth intensity by the
    // combined birth prior at the birth position (cand.mean is already set from
    // estimator.initiate(z) above) and compute the R1 phantom-birth gate
    // reference — same shared helper as buildNewTargetCandidates (one model
    // evaluation; obstacle-relaxed / land-kept semantics live in the helper).
    {
      const SuppressedBirth sb =
          applyBirthPriors(lambda_birth, lambda_z, birthPriorsAt(cand.mean));
      cand.rho_target = sb.rho_target;
      cand.rho_total = sb.rho_total;
      cand.rho_target_unsuppressed = sb.rho_target_unsuppressed;
      cand.rho_total_unsuppressed = sb.rho_total_unsuppressed;
    }

    out.push_back(std::move(cand));
  }

  return out;
}

void PmbmTracker::enumerateChildren(
    const GlobalHypothesis& parent,
    const std::vector<Measurement>& scan,
    const std::vector<NewTargetCandidate>& nts,
    std::vector<GlobalHypothesis>& out,
    int k_override,
    int parent_idx) {
  const int n = static_cast<int>(parent.bernoullis.size());
  const int m = static_cast<int>(scan.size());

  // Source-aware misdetection: collect this scan's source_ids, vessel_ids
  // (mmsi), and platform_ids once per scan.  Bernoullis whose
  // contribution-history identity is entirely absent from this set get a
  // no-op misdetection (state and r unchanged) — see
  // Config::source_aware_misdetection and ::source_aware_identity.
  std::set<std::string> scan_sources;
  std::set<std::uint32_t> scan_vessels;   // mmsi-keyed identity
  std::set<std::uint64_t> scan_platforms; // platform_id-keyed identity
  if (cfg_.source_aware_misdetection) {
    for (const auto& z : scan) {
      scan_sources.insert(z.source_id);
      if (cfg_.source_aware_identity && z.hints.mmsi.has_value())
        scan_vessels.insert(*z.hints.mmsi);
      if (cfg_.source_aware_identity && z.hints.platform_id.has_value())
        scan_platforms.insert(*z.hints.platform_id);
    }
  }
  // Unified identity gate: a touch is "covered" in this scan if it shares
  // EITHER mmsi (vessel_id) OR platform_id with a scan measurement.
  // When any identity key is known for a touch, the source_id channel
  // fallback is suppressed (continue) — prevents "broadcast-channel"
  // false observability between vessels that share only a source_id.
  auto should_misdetect = [&](BernoulliId id) {
    if (!cfg_.source_aware_misdetection) return true;
    auto it = contribution_history_.find(id);
    if (it == contribution_history_.end() || it->second.empty()) {
      return true;  // no prior history; treat as observable
    }
    for (const auto& touch : it->second) {
      if (cfg_.source_aware_identity) {
        if (touch.vessel_id.has_value()) {
          if (scan_vessels.count(*touch.vessel_id)) return true;   // matched by mmsi
        }
        if (touch.platform_id.has_value()) {
          if (scan_platforms.count(*touch.platform_id)) return true;  // matched by platform_id
        }
        if (touch.vessel_id.has_value() || touch.platform_id.has_value()) {
          continue;  // identity known but absent from scan → no coverage
        }
      }
      if (scan_sources.count(touch.source_id)) return true;  // channel fallback
    }
    return false;
  };

  // Idle-decay factor for the no-information branch (source-aware skip
  // OR out-of-any-sensor-coverage). Uses the Bernoulli's most-recent
  // contribution-touch time as "last observed". last_update can't be
  // used directly because the estimator's predict advances it to the
  // current filter time even when no measurement was applied.
  // Returns 1 (no decay) when the knob is disabled, no prior touch
  // history exists, or no time has elapsed.
  const double scan_time_sec =
      scan.empty() ? current_time_.seconds() : scan.front().time.seconds();
  auto idle_decay_for = [&](const Bernoulli& b) {
    if (cfg_.idle_halflife_sec <= 0.0) return 1.0;
    auto it = contribution_history_.find(b.id);
    if (it == contribution_history_.end() || it->second.empty()) return 1.0;
    double last_touch_s = -std::numeric_limits<double>::infinity();
    for (const auto& touch : it->second) {
      const double ts = touch.time.seconds();
      if (ts > last_touch_s) last_touch_s = ts;
    }
    const double dt = scan_time_sec - last_touch_s;
    if (dt <= 0.0) return 1.0;
    return std::exp(-std::log(2.0) * dt / cfg_.idle_halflife_sec);
  };

  // Per-measurement P_D under the detection model (cost-matrix and
  // detection log-weight). Config fallback when no model wired.
  std::vector<double> pD_l(m, cfg_.probability_of_detection);
  if (detection_model_) {
    for (int l = 0; l < m; ++l) {
      pD_l[l] = detection_model_->paramsFor(scan[l]).probability_of_detection;
    }
  }
  // Per-Bernoulli effective miss P_D under the detection model.
  // Aggregate across scan sensors via "1 − Π (1 − p_D_s · cov_s)" =
  // probability that at least one in-coverage scan sensor would have
  // detected this Bernoulli. Coverage = missDetectionProbability !=
  // 0 (the detection model returns 0 when the Bernoulli is outside
  // the sensor's max_range / sector). Config fallback when no model.
  //
  // With cfg_.dedup_miss_pd = true: dedup by (sensor, model, source_id)
  // so N returns from one radar sweep count as ONE detection opportunity
  // (textbook "one sweep = one detection opportunity" model). Off by
  // default (legacy per-measurement product, bit-identical to Phase 8/9
  // baseline). See Config::dedup_miss_pd.
  auto compute_miss_pD = [&](const Bernoulli& b) {
    if (!detection_model_) return cfg_.probability_of_detection;
    if (b.mean.size() < 2) return cfg_.probability_of_detection;
    const Eigen::Vector2d b_xy = b.mean.head<2>();
    double survive = 1.0;
    bool any_coverage = false;
    if (cfg_.dedup_miss_pd) {
      // Textbook: one detection opportunity per distinct sensor channel
      // that surveyed this scan, NOT one per return.
      std::set<std::tuple<SensorKind, MeasurementModel, std::string>> seen;
      for (const auto& z : scan) {
        auto key = std::make_tuple(z.sensor, z.model, z.source_id);
        if (!seen.insert(key).second) continue;  // already counted this channel
        const double pD_s = detection_model_->missDetectionProbability(
            z.sensor, z.model, b_xy, z.sensor_position_enu, z.source_id);
        if (pD_s > 0.0) { any_coverage = true; survive *= (1.0 - pD_s); }
      }
    } else {
      for (const auto& z : scan) {  // legacy per-measurement (buggy) path
        const double pD_s = detection_model_->missDetectionProbability(
            z.sensor, z.model, b_xy, z.sensor_position_enu, z.source_id);
        if (pD_s > 0.0) { any_coverage = true; survive *= (1.0 - pD_s); }
      }
    }
    return any_coverage ? (1.0 - survive) : 0.0;
  };

  // Edge case: empty scan. Only one child = parent with all-miss updates.
  // Misdetection: r ← (1 − p_D)·r / (1 − r + (1 − p_D)·r), state unchanged.
  // Hypothesis weight gains a Σ log(1 − r·p_D) shift (relative ordering
  // among parents preserved; absolute log_weight tracked for pruning).
  if (m == 0) {
    GlobalHypothesis child;
    child.weight = parent.weight;
    child.log_weight = parent.log_weight;
    child.bernoullis.reserve(parent.bernoullis.size());
    for (const auto& b : parent.bernoullis) {
      Bernoulli updated = b;
      // Empty-scan branch: no measurement → no update → post-update
      // state == post-predict state. Pass b.mean/b.covariance for
      // both predicted and filtered (smoother G ≈ I at this point).
      if (!should_misdetect(b.id)) {
        updated.existence_probability *= idle_decay_for(b);
        appendTrajectoryPoint(updated, cfg_.trajectory_window_scans,
                              current_time_, b.mean, b.covariance);
        child.bernoullis.push_back(std::move(updated));
        continue;
      }
      // Task 5: honest per-duty-cycle coverage model (spec §4).
      // When wired, replace compute_miss_pD/idle_decay_for with a single
      // evaluate() call — one miss charged only when the surveillance sensor
      // completed a full sweep over this track's predicted position.
      if (cfg_.use_sensor_activity && sensor_activity_ != nullptr) {
        auto ait = last_activity_check_.find(b.id);
        const Timestamp last_check =
            (ait != last_activity_check_.end()) ? ait->second : b.birth_time;
        const MissOpportunity opp = sensor_activity_->evaluate(
            b.mean.head<2>(), own_ship_enu_, b.mmsi, b.platform_id, last_check,
            current_time_);
        if (!opp.surveillance_miss) {
          // No surveillance opportunity (sensor mid-sweep, out of coverage,
          // or cooperative-only channel). Existence UNCHANGED by miss math.
          if (opp.cooperative_overdue) {
            // Task 6: cooperative own-identity report is overdue.
            // NEVER touch existence here (spec §9c: comms loss ≠ vessel sank).
            // Check cooperative-only timeout retirement: last known report is
            // b.birth_time (or last_cooperative_touch_ if detected before).
            auto tit = last_cooperative_touch_.find(b.id);
            const double last_report_sec =
                (tit != last_cooperative_touch_.end())
                    ? tit->second.seconds()
                    : b.birth_time.seconds();
            if (cfg_.cooperative_stale_timeout_sec > 0.0 &&
                current_time_.seconds() - last_report_sec >
                    cfg_.cooperative_stale_timeout_sec) {
              // Cooperative-only retirement: force existence to 0 so
              // pruneAndNormalise removes this Bernoulli. onTrackDeleted
              // fires via the lifecycle sink on the next scan. We do NOT
              // fire the stale signal here (retirement supersedes staleness).
              updated.existence_probability = 0.0;
            } else {
              // Still within timeout: mark stale but leave existence.
              cooperative_overdue_ids_.insert(b.id);
            }
          }
          appendTrajectoryPoint(updated, cfg_.trajectory_window_scans,
                                current_time_, b.mean, b.covariance);
          child.bernoullis.push_back(std::move(updated));
          continue;
        }
        // One completed sweep with no return: charge exactly one miss.
        // Deferred write: stage (not mutate) the window so every parent
        // sees the same pre-scan snapshot. Charging a miss resolves
        // observability up to current_time_, so the window advances there.
        stageActivityCheck(b.id, current_time_);
        const double r = b.existence_probability;
        const double pD = opp.p_D;
        const double miss_norm = 1.0 - r * pD;
        updated.existence_probability =
            (miss_norm > 0.0) ? ((1.0 - pD) * r) / miss_norm : 0.0;
        child.log_weight += std::log(std::max(miss_norm, 1e-300));
        appendTrajectoryPoint(updated, cfg_.trajectory_window_scans,
                              current_time_, b.mean, b.covariance);
        child.bernoullis.push_back(std::move(updated));
        continue;
      }
      // Legacy path (use_sensor_activity == false): unchanged, bit-identical.
      const double r = b.existence_probability;
      const double pD = compute_miss_pD(b);
      if (pD <= 0.0) {  // out of any sensor's coverage; no penalty
        updated.existence_probability *= idle_decay_for(b);
        appendTrajectoryPoint(updated, cfg_.trajectory_window_scans,
                              current_time_, b.mean, b.covariance);
        child.bernoullis.push_back(std::move(updated));
        continue;
      }
      const double miss_norm = 1.0 - r * pD;
      updated.existence_probability =
          (miss_norm > 0.0) ? ((1.0 - pD) * r) / miss_norm : 0.0;
      child.log_weight += std::log(std::max(miss_norm, 1e-300));
      appendTrajectoryPoint(updated, cfg_.trajectory_window_scans,
                            current_time_, b.mean, b.covariance);
      child.bernoullis.push_back(std::move(updated));
    }
    out.push_back(std::move(child));
    return;
  }

  // Build cost matrix C: (n + m) × m.
  // - C[i, l]      = −log(r_i · ℓ_{i,l})            i = 0..n−1  (Bernoulli ↔ z_l)
  // - C[n+l', l]   = −log(ρ_total_l) iff l == l'                (new target absorbs z_l)
  //                  +∞ otherwise
  //
  // We also pre-compute the per-cell updated Bernoulli (state, cov) and
  // the per-cell log-likelihood, so the cheap K-best path doesn't redo
  // estimator.update.
  Eigen::MatrixXd C =
      Eigen::MatrixXd::Constant(n + m, m, kInf);
  std::vector<std::vector<double>> log_lik(n, std::vector<double>(m, -kInf));
  std::vector<std::vector<Bernoulli>> updated(
      n, std::vector<Bernoulli>(m));

  // Phase 8 P1: pre-gate before the expensive estimator.update.
  // MATLAB MTT-master gates on Mahalanobis before filling cost cells;
  // we skip update for any cell whose Mahalanobis fails the configured
  // gate_threshold. Saves the per-cell update (~70% of cells on dense
  // clutter, ~30% on autoferry). The skipped cell's cost stays +∞ so
  // the assignment treats it as infeasible.
  for (int i = 0; i < n; ++i) {
    const Bernoulli& b = parent.bernoullis[i];
    if (b.existence_probability <= 0.0) continue;
    Track t = toTrack(b);
    for (int l = 0; l < m; ++l) {
      if (cfg_.gate_threshold > 0.0 &&
          !estimator_.gate(t, scan[l], cfg_.gate_threshold)) {
        // Cell stays at kInf in C; log_lik stays at -kInf; updated[i][l]
        // left default-constructed (it can never be referenced by the
        // assignment because the cost is +∞).
        continue;
      }
      const double ll = estimator_.logLikelihood(t, scan[l]);
      log_lik[i][l] = ll;
      Track t_upd = t;
      estimator_.update(t_upd, scan[l]);
      Bernoulli upd = b;
      fromTrack(upd, t_upd);
      upd.existence_probability = 1.0;  // existence given detection
      // #25 Phase 2b: the TRUE raw position innovation of measurement l against
      // the predicted position (b.mean is post-predict/pre-update). Position-
      // model measurements carry an ENU position directly; bearing-only carry
      // none → sentinel. Computed when a diagnostic sink is attached OR the
      // velocity-runaway guard is armed (it keys on this innovation); otherwise
      // skipped → byte-identical. It is never fed back into the update math here
      // (the guard treatment below is applied to the finalised detected
      // Bernoulli, post-assignment/PDA).
      if (diag_sink_ != nullptr || cfg_.innov_gate_max_m > 0.0) {
        const MeasurementModel mdl = scan[l].model;
        if ((mdl == MeasurementModel::Position2D ||
             mdl == MeasurementModel::PositionVelocity2D) &&
            scan[l].value.size() >= 2 && b.mean.size() >= 2) {
          upd.last_innovation_enu = scan[l].value.head<2>() - b.mean.head<2>();
          upd.last_innovation_norm_m = upd.last_innovation_enu.norm();
        } else {
          upd.last_innovation_enu.setZero();
          upd.last_innovation_norm_m = -1.0;
        }
      }
      updated[i][l] = std::move(upd);
      const double r = b.existence_probability;
      const double cost_val = -(std::log(r) + ll);
      C(i, l) = std::isfinite(cost_val) ? cost_val : kInf;
    }
  }

  // New-target rows: diagonal block.
  for (int l = 0; l < m; ++l) {
    const double rho = nts[l].rho_total;
    if (rho > 0.0) {
      C(n + l, l) = -std::log(rho);
    }
  }

  // Murty K-best on the cost matrix.
  // k_override (Phase 8 P2): adaptive K per parent, weight-proportional.
  // Falls back to cfg_.k_best_per_hypothesis when k_override < 0.
  const int k_effective = (k_override >= 1)
      ? k_override
      : std::max(1, cfg_.k_best_per_hypothesis);
  const KBestResult kb = murtyKBest(C, k_effective);

  for (std::size_t k = 0; k < kb.assignments.size(); ++k) {
    const auto& asn = kb.assignments[k];
    // Validity check: any assignment cell that hit the BIG_M padding
    // (cost ≥ kInf marker; in practice, any +∞ in the original matrix)
    // would yield an invalid child. Skip.
    bool feasible = true;
    for (int row = 0; row < static_cast<int>(asn.size()); ++row) {
      const int col = asn[row];
      if (col < 0) continue;  // unassigned row, no contribution
      if (!std::isfinite(C(row, col))) {
        feasible = false;
        break;
      }
    }
    if (!feasible) continue;

    GlobalHypothesis child;
    child.bernoullis.reserve(n + m);  // upper bound
    child.log_weight = parent.log_weight;

    // First: which Bernoullis got an assignment? Mark for misdetection
    // pass.
    std::vector<int> bernoulli_to_meas(n, -1);
    for (int row = 0; row < n; ++row) {
      const int col = asn[row];
      if (col >= 0 && col < m && std::isfinite(C(row, col))) {
        bernoulli_to_meas[row] = col;
      }
    }
    // PDA soft update (default off): a measurement claimed by ANOTHER existing
    // Bernoulli in this child must not enter a track's soft-update pool (that
    // would double-count a shared return → over-count). Measurements that are
    // unclaimed or claimed only by a new-target birth row may (they were clutter
    // / a fresh birth anyway). Built once per child.
    std::vector<char> claimed_by_existing(m, 0);
    if (cfg_.use_pda_soft_detected_branch)
      for (int row = 0; row < n; ++row)
        if (bernoulli_to_meas[row] >= 0)
          claimed_by_existing[bernoulli_to_meas[row]] = 1;

    // Detected Bernoullis.
    for (int i = 0; i < n; ++i) {
      const Bernoulli& b = parent.bernoullis[i];
      if (bernoulli_to_meas[i] >= 0) {
        const int l = bernoulli_to_meas[i];
        Bernoulli det = updated[i][l];
        // PDA soft detected-branch update (default off): rather than commit the
        // state fully to the assigned winner l, β-weight the per-cell updates
        // over the pool { l } ∪ { gated j not claimed by another existing
        // Bernoulli }. β_j ∝ p_D(j)·ℓ_{i,j} (detections-only). Moment-match with
        // the innovation-spread term. Only the STATE changes — the hypothesis
        // weight (below) still uses the winner l, so K=1 / Murty are untouched.
        // Pool size 1 → unchanged hard update. This defeats a gate-closer
        // clutter dragging a real open-sea track off-target.
        if (cfg_.use_pda_soft_detected_branch &&
            (!cfg_.pda_soft_detected_branch_on_confirmed_only ||
             b.existence_probability >= cfg_.output_existence_floor)) {
          std::vector<int> pool;
          double max_s = -kInf;
          for (int j = 0; j < m; ++j) {
            if (!std::isfinite(C(i, j))) continue;            // not gated
            if (j != l && claimed_by_existing[j]) continue;   // owned by another track
            // Land-aware pool: keep shore/structure returns out of the softening
            // pool so PDA blends against WATER clutter only (the winner l is
            // always kept — the hard-assignment decision is unchanged). Inert
            // when off or no land model → byte-identical to the plain pool.
            if (j != l && cfg_.pda_pool_excludes_land && land_model_ != nullptr &&
                land_model_->clutterPrior(updated[i][j].mean.head<2>()) >
                    cfg_.pda_pool_land_clutter_gate)
              continue;
            pool.push_back(j);
            const double s = std::log(pD_l[j]) + log_lik[i][j];
            if (s > max_s) max_s = s;
          }
          if (pool.size() > 1) {
            std::vector<double> w(pool.size());
            double wsum = 0.0;
            for (std::size_t p = 0; p < pool.size(); ++p) {
              const int j = pool[p];
              w[p] = pD_l[j] * std::exp(log_lik[i][j] - max_s);
              wsum += w[p];
            }
            if (wsum > 0.0) {
              for (double& wp : w) wp /= wsum;
              // Mode-collapsed mean/cov with the PDA spread-of-innovations term.
              Eigen::VectorXd mean = Eigen::VectorXd::Zero(det.mean.size());
              for (std::size_t p = 0; p < pool.size(); ++p)
                mean += w[p] * updated[i][pool[p]].mean;
              Eigen::MatrixXd cov =
                  Eigen::MatrixXd::Zero(det.covariance.rows(), det.covariance.cols());
              for (std::size_t p = 0; p < pool.size(); ++p) {
                const Eigen::VectorXd d = updated[i][pool[p]].mean - mean;
                cov += w[p] * (updated[i][pool[p]].covariance + d * d.transpose());
              }
              det.mean = std::move(mean);
              det.covariance = std::move(cov);
              // IMM: β-combine each mode's mean/cov/probability likewise.
              if (det.imm_means.cols() > 0) {
                const int K = static_cast<int>(det.imm_means.cols());
                const int sd = static_cast<int>(det.imm_means.rows());
                Eigen::MatrixXd im = Eigen::MatrixXd::Zero(sd, K);
                Eigen::VectorXd mp = Eigen::VectorXd::Zero(K);
                for (std::size_t p = 0; p < pool.size(); ++p) {
                  const Bernoulli& u = updated[i][pool[p]];
                  im += w[p] * u.imm_means;
                  mp += w[p] * u.imm_mode_probabilities;
                }
                std::vector<Eigen::MatrixXd> ic(
                    K, Eigen::MatrixXd::Zero(sd, sd));
                for (int mode = 0; mode < K; ++mode)
                  for (std::size_t p = 0; p < pool.size(); ++p) {
                    const Bernoulli& u = updated[i][pool[p]];
                    const Eigen::VectorXd d = u.imm_means.col(mode) - im.col(mode);
                    ic[mode] += w[p] * (u.imm_covariances[mode] + d * d.transpose());
                  }
                det.imm_means = std::move(im);
                det.imm_mode_probabilities = std::move(mp);
                det.imm_covariances = std::move(ic);
              }
            }
          }
        }
        // W5.2: stamp the state's PHYSICAL time, not the claimed measurement's
        // time. predict() advanced every component to current_time_ (== t_max,
        // the latest timestamp in this batch) before enumeration, and the EKF
        // update does NOT re-predict — so the detected child's mean/covariance
        // live at current_time_. In a mixed-timestamp scan scan[l].time can be
        // < t_max; stamping it there rewinds last_update below the state's real
        // time, and the next predict then applies F/Q over [scan[l].time, t_max]
        // a SECOND time (double-counted process noise). On a uniform scan
        // scan[l].time == current_time_, so this is byte-identical (every bench
        // + existing test feeds uniform scans).
        //
        // W5.2<->F2 composition (this branch rebased onto master after the F2
        // cycle): the SourceTouch provenance walk near line 1693 was re-keyed by
        // F2 onto det.last_claimed_meas_index (set on the next line), which is
        // independent of last_update. So stamping last_update at t_max here no
        // longer affects attribution — the earlier concern (that a MIXED-timestamp
        // scan would break the old `z.time == b.last_update` key) is moot; the two
        // fixes are orthogonal. Uniform scans (all benches/tests) are unaffected by
        // either. The mixed-ts positive-attribution case is pinned by
        // test_pmbm_contribution_provenance's MixedTimestampMultiSensor... test.
        det.last_update = current_time_;
        det.last_claimed_meas_index = l;  // R2: true assignment for the feed
        // #25 Phase 2b velocity-runaway guard (default OFF, innov_gate_max_m<=0).
        // The association is already decided (this cell won the assignment) —
        // we ACCEPT the measurement's position but, when its position innovation
        // is grossly oversized, treat the corrupted velocity/turn-rate so the
        // track cannot fly off. Kinematic only: r/mass/id/existence untouched.
        // det.last_innovation_norm_m was computed above (>=0 only for a
        // position-model detection; sentinel -1 never exceeds a positive gate).
        if (cfg_.innov_gate_max_m > 0.0 &&
            det.last_innovation_norm_m > cfg_.innov_gate_max_m) {
          applyInnovationGate(det);
        }
        // R11 identity surfacing: last-write-wins from the claimed measurement's
        // hints. `det` was copied from the parent Bernoulli (line above), so an
        // identity-free claim (e.g. a radar plot) preserves a previously-seen
        // mmsi/platform_id; an AIS/cooperative claim refreshes it.
        if (scan[l].hints.mmsi.has_value()) det.mmsi = scan[l].hints.mmsi;
        if (scan[l].hints.platform_id.has_value())
          det.platform_id = scan[l].hints.platform_id;
        if (scan[l].hints.heading_deg.has_value())
          det.heading_deg = scan[l].hints.heading_deg;
        if (scan[l].hints.nav_status.has_value())
          det.nav_status = scan[l].hints.nav_status;
        // Detection: x_pred = parent's state (post-predict, pre-update);
        //            x_filt = updated state.
        appendTrajectoryPoint(det, cfg_.trajectory_window_scans,
                              scan[l].time, b.mean, b.covariance);
        child.bernoullis.push_back(std::move(det));
        // Task 6 / channel-kind fix: a detection from ANY sensor declared as
        // ChannelKind::Cooperative in the activity profile resets the
        // cooperative-only retirement timer. This includes SensorKind::Ais
        // (cooperative-announce: silence is weak evidence keyed on identity)
        // when the provider declares AIS as Cooperative. Falls back to the
        // legacy `== SensorKind::Cooperative` check when no profile is wired,
        // preserving bit-identical behaviour.
        // Deferred write (staged): keeps the retirement-timeout read snapshot
        // hypothesis-consistent across parents.
        if (isCooperativeSource(scan[l].sensor))
          stageCooperativeTouch(b.id, scan[l].time);
        // Task 5 Step-4 fix (defect #1): a detection resolves observability
        // up to scan[l].time, so it ADVANCES the surveillance-miss window.
        // Without this, the first dropout after a normal tracking run sees
        // dt = now - birth_time (or last miss) and wrongly charges a full
        // miss even though only one short scan interval elapsed since the
        // detection. Staged (deferred) + gated so it is inert in legacy mode.
        if (cfg_.use_sensor_activity && sensor_activity_ != nullptr)
          stageActivityCheck(b.id, scan[l].time);
        // Full per-Bernoulli detection contribution to log-weight:
        // log(r · p_D · ℓ).  P_D is the assigned measurement's
        // per-(sensor, source) P_D under the detection model.
        child.log_weight +=
            std::log(b.existence_probability) + std::log(pD_l[l]) +
            log_lik[i][l];
      } else {
        // Misdetection: r ← (1 − p_D) · r / (1 − r·p_D), state unchanged.
        // Skip the recursion entirely when source-aware misdetection
        // says no sensor in this scan could have observed this
        // Bernoulli (e.g. AIS broadcast from a different vessel) —
        // but still apply the idle-decay (knob: idle_halflife_sec) so
        // ghost Bernoullis whose target has stopped reporting do
        // eventually fall below r_min.
        Bernoulli miss = b;
        miss.last_claimed_meas_index = -1;  // R2: claimed nothing this scan
        // #25 P2b: no measurement applied → clear the parent's stale innovation.
        if (diag_sink_ != nullptr) {
          miss.last_innovation_enu.setZero();
          miss.last_innovation_norm_m = -1.0;
        }
        // Misdetection: no update at this scan → predicted == filtered.
        if (!should_misdetect(b.id)) {
          miss.existence_probability *= idle_decay_for(b);
          appendTrajectoryPoint(miss, cfg_.trajectory_window_scans,
                                current_time_, b.mean, b.covariance);
          child.bernoullis.push_back(std::move(miss));
          continue;
        }
        // Task 5: honest per-duty-cycle coverage model (spec §4).
        if (cfg_.use_sensor_activity && sensor_activity_ != nullptr) {
          auto ait = last_activity_check_.find(b.id);
          const Timestamp last_check =
              (ait != last_activity_check_.end()) ? ait->second : b.birth_time;
          const MissOpportunity opp = sensor_activity_->evaluate(
              b.mean.head<2>(), own_ship_enu_, b.mmsi, b.platform_id,
              last_check, current_time_);
          if (!opp.surveillance_miss) {
            // No surveillance opportunity (mid-sweep, out of coverage,
            // or cooperative-only channel). Existence UNCHANGED by miss math.
            if (opp.cooperative_overdue) {
              // Task 6: cooperative own-identity report is overdue.
              // NEVER touch existence (spec §9c). Check timeout retirement.
              auto tit = last_cooperative_touch_.find(b.id);
              const double last_report_sec =
                  (tit != last_cooperative_touch_.end())
                      ? tit->second.seconds()
                      : b.birth_time.seconds();
              if (cfg_.cooperative_stale_timeout_sec > 0.0 &&
                  current_time_.seconds() - last_report_sec >
                      cfg_.cooperative_stale_timeout_sec) {
                // Retire: force existence to 0 → pruned by r_min.
                miss.existence_probability = 0.0;
              } else {
                cooperative_overdue_ids_.insert(b.id);
              }
            }
            appendTrajectoryPoint(miss, cfg_.trajectory_window_scans,
                                  current_time_, b.mean, b.covariance);
            child.bernoullis.push_back(std::move(miss));
            continue;
          }
          // One completed sweep with no return: charge exactly one miss.
          // Deferred write: stage (not mutate) the window so every parent
          // reads the same pre-scan snapshot (hypothesis-consistent).
          stageActivityCheck(b.id, current_time_);
          const double r = b.existence_probability;
          const double pD = opp.p_D;
          const double miss_norm = 1.0 - r * pD;
          miss.existence_probability =
              (miss_norm > 0.0) ? ((1.0 - pD) * r) / miss_norm : 0.0;
          // Misdetection contribution: log(1 − r · p_D).
          child.log_weight += std::log(std::max(miss_norm, 1e-300));
          appendTrajectoryPoint(miss, cfg_.trajectory_window_scans,
                                current_time_, b.mean, b.covariance);
          child.bernoullis.push_back(std::move(miss));
          continue;
        }
        // Legacy path (use_sensor_activity == false): unchanged, bit-identical.
        const double r = b.existence_probability;
        const double pD = compute_miss_pD(b);
        if (pD <= 0.0) {  // out of any sensor's coverage; no penalty
          miss.existence_probability *= idle_decay_for(b);
          appendTrajectoryPoint(miss, cfg_.trajectory_window_scans,
                                current_time_, b.mean, b.covariance);
          child.bernoullis.push_back(std::move(miss));
          continue;
        }
        const double miss_norm = 1.0 - r * pD;
        miss.existence_probability =
            (miss_norm > 0.0) ? ((1.0 - pD) * r) / miss_norm : 0.0;
        // Misdetection contribution: log(1 − r · p_D).
        child.log_weight += std::log(std::max(miss_norm, 1e-300));
        appendTrajectoryPoint(miss, cfg_.trajectory_window_scans,
                              current_time_, b.mean, b.covariance);
        child.bernoullis.push_back(std::move(miss));
      }
    }

    // Measurements claimed by new-target rows → new Bernoullis.
    for (int row = n; row < n + m; ++row) {
      const int col = asn[row];
      if (col < 0 || col >= m) continue;
      if (!std::isfinite(C(row, col))) continue;
      const int l = col;
      const auto& nt = nts[l];
      const double r_new =
          (nt.rho_total > 0.0) ? (nt.rho_target / nt.rho_total) : 0.0;
      // R1 phantom-birth gate: check the UNSUPPRESSED existence (finding #3:
      // obstacle relaxed, land kept), not the suppressed r_new. Soft geographic
      // suppression must not compose with min_new_bernoulli_existence into a hard
      // no-birth zone. A birth whose gate existence clears the floor still
      // materialises with the SUPPRESSED r_new below. It then accumulates on
      // re-detection PROVIDED that suppressed r_new clears r_min (1e-3); a target
      // driven below r_min by deep land×obstacle composition is pruned the same
      // scan and is NOT trackable from position alone — by design (finding #1).
      // Forcing persistence by flooring existence to r_min would seed phantom
      // tracks in clutter; the real cure for that regime is sensor-aware
      // suppression (ADR 0001 A3). A genuine phantom (gate existence below the
      // floor) and a hard-drop (birthScale < 0) are still killed. Bit-identical
      // when no model is wired (unsuppressed == suppressed). The measurement
      // still consumes ρ_total mass so the assignment stays balanced.
      const double r_new_gate =
          (nt.rho_total_unsuppressed > 0.0)
              ? (nt.rho_target_unsuppressed / nt.rho_total_unsuppressed)
              : 0.0;
      if (r_new_gate < cfg_.min_new_bernoulli_existence) {
        child.log_weight += std::log(std::max(nt.rho_total, 1e-300));
        continue;
      }
      Bernoulli nb;
      // Phase 8 iter 5 birth-id cache: under adaptive K, siblings of
      // the same parent that birth a Bernoulli for the same measurement
      // share an id so the within-hypothesis merge and weight prune
      // see one logical hypothesis rather than 5 distinct ids.
      // parent_idx < 0 disables caching (legacy bit-identical).
      //
      // Phase 9 cross-parent extension: when
      // cfg_.cross_parent_birth_id_cache is true, the cache key drops
      // parent_idx (use -1 as the parent slot) so ALL parents' K-
      // children birthing the same measurement share one id. Mirrors
      // MATLAB filter-level new-track creation.
      if (parent_idx >= 0) {
        const int key_parent = cfg_.cross_parent_birth_id_cache
                                   ? -1
                                   : parent_idx;
        const auto key = std::make_pair(key_parent, l);
        auto cit = scan_birth_id_cache_.find(key);
        if (cit != scan_birth_id_cache_.end()) {
          nb.id = cit->second;
        } else {
          nb.id = next_bernoulli_id_++;
          scan_birth_id_cache_[key] = nb.id;
        }
      } else {
        nb.id = next_bernoulli_id_++;
      }
      nb.existence_probability = r_new;
      nb.mean = nt.mean;
      nb.covariance = nt.covariance;
      nb.imm_means = nt.imm_means;
      nb.imm_covariances = nt.imm_covariances;
      nb.imm_mode_probabilities = nt.imm_mode_probabilities;
      nb.last_update = nt.state_time;  // W5.2: physical state time (path-specific:
                                       // t_max for PPP, z.time for adaptive birth)
      nb.last_claimed_meas_index = l;  // R2: this measurement birthed it
      // #25 P2b: a birth has no predicted position to innovate against → sentinel.
      if (diag_sink_ != nullptr) {
        nb.last_innovation_enu.setZero();
        nb.last_innovation_norm_m = -1.0;
      }
      nb.birth_time = scan[l].time;
      // R11 identity surfacing: seed identity from the birthing measurement's
      // hints (nullopt if it carries none — refined by later claims). #20 adds
      // the target-reported attributes on the same footing.
      nb.mmsi = scan[l].hints.mmsi;
      nb.platform_id = scan[l].hints.platform_id;
      nb.heading_deg = scan[l].hints.heading_deg;
      nb.nav_status = scan[l].hints.nav_status;
      // Birth: no prior → predicted = filtered (smoother G ≈ I at
      // first point, no effect).
      appendTrajectoryPoint(nb, cfg_.trajectory_window_scans,
                            scan[l].time, nb.mean, nb.covariance);
      child.bernoullis.push_back(std::move(nb));
      // New-target contribution: log(ρ_total).
      child.log_weight += std::log(nt.rho_total);
    }

    out.push_back(std::move(child));
  }
}

void PmbmTracker::pruneAndNormalise() {
  // Normalise mixture weights from log_weight (numerically stable).
  if (density_.mbm.empty()) return;

  std::vector<double> lws;
  lws.reserve(density_.mbm.size());
  for (const auto& h : density_.mbm) lws.push_back(h.log_weight);
  const double lse = logSumExp(lws);
  if (std::isfinite(lse)) {
    for (auto& h : density_.mbm) {
      h.weight = std::exp(h.log_weight - lse);
    }
  } else {
    // All -inf: degenerate, distribute uniformly to keep the filter alive.
    const double u = 1.0 / static_cast<double>(density_.mbm.size());
    for (auto& h : density_.mbm) h.weight = u;
  }

  // Drop hypotheses below the weight floor.
  std::size_t diag_n_before_floor = 0;
  if (diag_sink_ != nullptr) diag_n_before_floor = density_.mbm.size();
  density_.mbm.erase(
      std::remove_if(density_.mbm.begin(), density_.mbm.end(),
                     [&](const GlobalHypothesis& h) {
                       return h.weight < cfg_.hypothesis_weight_min;
                     }),
      density_.mbm.end());
  if (diag_sink_ != nullptr)
    diag_hyp_dropped_floor_ +=
        static_cast<int>(diag_n_before_floor - density_.mbm.size());

  // Cap mixture size at max_global_hypotheses (keep top-weighted).
  if (density_.mbm.size() > cfg_.max_global_hypotheses) {
    if (diag_sink_ != nullptr)
      diag_hyp_dropped_cap_ +=
          static_cast<int>(density_.mbm.size() - cfg_.max_global_hypotheses);
    std::partial_sort(
        density_.mbm.begin(),
        density_.mbm.begin() + cfg_.max_global_hypotheses,
        density_.mbm.end(),
        [](const GlobalHypothesis& a, const GlobalHypothesis& b) {
          return a.weight > b.weight;
        });
    density_.mbm.resize(cfg_.max_global_hypotheses);
  }

  // Renormalise after pruning.
  double s = 0.0;
  for (const auto& h : density_.mbm) s += h.weight;
  if (s > 0.0) {
    for (auto& h : density_.mbm) h.weight /= s;
  } else if (!density_.mbm.empty()) {
    const double u = 1.0 / static_cast<double>(density_.mbm.size());
    for (auto& h : density_.mbm) h.weight = u;
  }

  // Within-hypothesis Bernoulli pruning by r_min.
  for (auto& h : density_.mbm) {
    const std::size_t nb = (diag_sink_ != nullptr) ? h.bernoullis.size() : 0;
    h.bernoullis.erase(
        std::remove_if(h.bernoullis.begin(), h.bernoullis.end(),
                       [&](const Bernoulli& b) {
                         return b.existence_probability < cfg_.r_min;
                       }),
        h.bernoullis.end());
    if (diag_sink_ != nullptr)
      diag_bernoulli_pruned_rmin_ +=
          static_cast<int>(nb - h.bernoullis.size());
  }

  // PPP weight pruning.
  density_.ppp.erase(
      std::remove_if(density_.ppp.begin(), density_.ppp.end(),
                     [&](const PoissonComponent& c) {
                       return c.weight < cfg_.weight_min;
                     }),
      density_.ppp.end());

  // PPP count cap. Without it, every clutter measurement under
  // measurement-driven birth seeds a component that lives for several
  // scans, growing the PPP unbounded on long replays. Keep top-weighted.
  if (density_.ppp.size() > cfg_.max_ppp_components) {
    std::partial_sort(
        density_.ppp.begin(),
        density_.ppp.begin() + cfg_.max_ppp_components,
        density_.ppp.end(),
        [](const PoissonComponent& a, const PoissonComponent& b) {
          return a.weight > b.weight;
        });
    density_.ppp.resize(cfg_.max_ppp_components);
  }
}

void PmbmTracker::stageActivityCheck(BernoulliId id, Timestamp t) {
  auto [it, inserted] = staged_activity_check_.insert({id, t});
  if (!inserted && t.seconds() > it->second.seconds()) it->second = t;
}

void PmbmTracker::stageCooperativeTouch(BernoulliId id, Timestamp t) {
  auto [it, inserted] = staged_cooperative_touch_.insert({id, t});
  if (!inserted && t.seconds() > it->second.seconds()) it->second = t;
}

void PmbmTracker::mergeStagedActivityMaps() {
  // Deferred write: advance the persistent windows to this scan's resolved
  // times. Applied once, after all parents are enumerated, so the read
  // snapshot was identical for every parent (hypothesis-order-independent).
  for (const auto& [id, t] : staged_activity_check_) last_activity_check_[id] = t;
  for (const auto& [id, t] : staged_cooperative_touch_)
    last_cooperative_touch_[id] = t;
}

void PmbmTracker::processBatch(const std::vector<Measurement>& scan_arg) {
  // Backlog #15: order the batch by time so callers need not pre-sort. PMBM is
  // already largely order-robust (predict advances to t_max below; association is
  // set-wise), so this is a no-op for the tracked state on default configs — it
  // makes the contract uniform with MhtTracker and pins the optional idle-decay
  // knob (which reads scan.front().time) to the earliest instant deterministically.
  // stable_sort keeps it deterministic; the is_sorted fast-path makes an
  // already-sorted batch a true no-op (bit-identical — every existing test/bench).
  std::vector<Measurement> scan_ordered;
  const auto by_time = [](const Measurement& a, const Measurement& b) {
    return a.time < b.time;
  };
  const bool need_sort =
      !std::is_sorted(scan_arg.begin(), scan_arg.end(), by_time);
  if (need_sort) {
    scan_ordered = scan_arg;
    std::stable_sort(scan_ordered.begin(), scan_ordered.end(), by_time);
  }
  const std::vector<Measurement>& scan_in = need_sort ? scan_ordered : scan_arg;
  // #28 (backlog #1's PMBM face): reject an out-of-order (stale) batch instead
  // of letting predict()'s dt<=0 branch rewind current_time_ and return without
  // propagating (the update would then run against newer states). Mirrors the
  // MhtTracker guard (Tracker.cpp:162 / MhtTracker.cpp:228 shape): the high-water
  // key is the batch FRONT (earliest instant, post-sort), the same convention as
  // the sibling trackers even though predict() advances to t_max. Default on;
  // deterministic in-order replay never trips it (bit-identical).
  //
  // Forced divergence from MhtTracker (documented in docs/algorithms/pmbm-design.md):
  // MhtTracker/Tracker early-return on an EMPTY batch; PMBM instead processes an
  // empty scan as an all-miss step (it advances the filter and applies
  // misdetection to every Bernoulli). An empty batch has no instant to compare,
  // so the guard is scoped to non-empty batches; the empty-scan path is
  // unchanged.
  if (!scan_in.empty()) {
    const Timestamp t_front = scan_in.front().time;
    if (has_high_water_ && t_front < high_water_) {
      if (cfg_.reject_stale_measurements) {
        stale_dropped_ += scan_in.size();
        return;
      }
    } else {
      high_water_ = t_front;
      has_high_water_ = true;
    }
  }
  // Task 6: reset per-scan stale set. enumerateChildren will populate it.
  cooperative_overdue_ids_.clear();
  // Backlog #25: reset per-scan structural-event counters (diagnostic only).
  if (diag_sink_ != nullptr) {
    diag_hyp_dropped_floor_ = 0;
    diag_hyp_dropped_cap_ = 0;
    diag_bernoulli_pruned_rmin_ = 0;
  }
  // Task 5 Step-4 fix: reset the per-scan staging maps. enumerateChildren
  // reads the persistent maps as a frozen snapshot and writes resolved
  // window times here; merged into the persistent maps after all parents.
  staged_activity_check_.clear();
  staged_cooperative_touch_.clear();
  // Apply per-sensor registration bias correction (item 9) + Schmidt-KF
  // R-inflation before any predict / update. Null provider =
  // bit-identical to legacy.
  std::vector<Measurement> scan_corrected;
  if (bias_provider_ != nullptr) {
    scan_corrected.reserve(scan_in.size());
    for (const auto& z : scan_in) {
      scan_corrected.push_back(applyBiasCorrection(z, bias_provider_));
    }
  }
  const std::vector<Measurement>& scan =
      bias_provider_ != nullptr ? scan_corrected : scan_in;
  // Backlog #25 diagnostic: record this scan's return count (post sort).
  if (diag_sink_ != nullptr)
    diag_last_scan_meas_count_ = static_cast<int>(scan.size());

  // W2.4a: capture own-ship's ENU position for the sensor-activity coverage
  // check. The declared surveillance profiles are own-ship-mounted, so their
  // sensor_position_enu is own-ship's ENU at scan time. Take the last
  // surveillance measurement (deterministic in the time-sorted scan) and
  // persist it across scans so the empty-scan misdetection branch retains
  // own-ship's last-known position. Gate matches the two evaluate() READ sites
  // (use_sensor_activity && sensor_activity_): a config that wires a provider
  // for channelKindFor alone (use_sensor_activity=false) does no dead work.
  //
  // Known limitation (improve-next): own-ship is inferred from surveillance
  // returns, not the authoritative OwnShipProvider pose. During a surveillance
  // dropout longer than the duty cycle on a moving platform it goes stale (and
  // is (0,0) before the first return); bounded, and inert on the deployable
  // replays (continuous radar), but a track near the coverage boundary can be
  // mis-charged. Threading OwnShipPose into the tracker would close it.
  if (cfg_.use_sensor_activity && sensor_activity_ != nullptr) {
    for (const auto& z : scan) {
      const auto kind = sensor_activity_->channelKindFor(z.sensor);
      if (kind == ChannelKind::Surveillance)
        own_ship_enu_ = z.sensor_position_enu;
    }
  }

  // Predict to the latest measurement time in the scan (matches the
  // MhtTracker convention). Empty scan still advances the filter if a
  // future predict() is wired separately; here it's just a no-op for
  // both predict and update.
  if (!scan.empty()) {
    Timestamp t_max = scan.front().time;
    for (const auto& z : scan) {
      if (z.time.seconds() > t_max.seconds()) t_max = z.time;
    }
    predict(t_max);
  }

  if (scan.empty()) {
    // Apply misdetection to all Bernoullis (no measurements means every
    // Bernoulli was missed). Walks through enumerateChildren with m=0
    // for each parent to keep one code path.
    std::vector<GlobalHypothesis> children;
    children.reserve(density_.mbm.size());
    const std::vector<NewTargetCandidate> empty_nts;
    if (density_.mbm.empty()) {
      // No prior hypotheses: seed the MBM with the empty hypothesis so
      // future Bernoullis (from PPP births) have somewhere to land.
      GlobalHypothesis seed;
      seed.weight = 1.0;
      seed.log_weight = 0.0;
      density_.mbm.push_back(std::move(seed));
    } else {
      for (const auto& p : density_.mbm) {
        enumerateChildren(p, scan, empty_nts, children);
      }
      density_.mbm = std::move(children);
    }
    // Task 5 Step-4 fix: apply the deferred window writes once, after every
    // parent has been enumerated against the frozen pre-scan snapshot.
    mergeStagedActivityMaps();
    // Phase 8 iter 4: empty-scan branch must also run the
    // within-hypothesis Bernoulli merge and fire lifecycle events.
    // Previously the early-return on line ~850 skipped both, so a
    // Bernoulli that decayed below output_existence_floor on an
    // empty scan never fired onTrackDeleted, and a re-promotion on
    // the *next* non-empty scan saw stale prev_emitted_statuses_.
    if (cfg_.bhattacharyya_merge_threshold > 0.0) {
      for (auto& h : density_.mbm) {
        mergeBernoulliDuplicates(h);
      }
    }
    pruneAndNormalise();
    aggregated_tracks_dirty_ = true;
    // Task 6: fire stale signals for cooperative-overdue tracks.
    if (stale_sink_ != nullptr && !cooperative_overdue_ids_.empty()) {
      const auto& ts = tracks();  // triggers refreshAggregatedTracks (Coasting)
      for (const auto& t : ts) {
        if (cooperative_overdue_ids_.count(t.id.value)) {
          stale_sink_->onTrackStale(t.id, current_time_);
        }
      }
    }
    if (track_sink_ != nullptr) {
      firePmbmLifecycleEvents(current_time_);
    }
    if (diag_sink_ != nullptr) {
      emitPmbmDiagnostics(current_time_);
      ++diag_scan_counter_;
    }
    return;
  }

  // Seed MBM with the empty hypothesis on first update (no prior
  // detected targets) so the per-parent loop has something to extend.
  if (density_.mbm.empty()) {
    GlobalHypothesis seed;
    seed.weight = 1.0;
    seed.log_weight = 0.0;
    density_.mbm.push_back(std::move(seed));
  }

  // Measurement-driven birth: append one PPP component per initiable
  // measurement BEFORE building new-target candidates, so a fresh
  // measurement that no Bernoulli claims has somewhere to live (and
  // ρ_target is dominated by the just-birthed component centred on the
  // measurement). Bearing-only measurements cannot initiate a state
  // and are skipped, matching MhtTracker birth-on-measurement rules.
  //
  // Skipped entirely under Adaptive Birth (Reuter 2014) — the
  // measurement-driven injection contaminates ρ_target by construction
  // (just-injected component dominates), which is exactly what Adaptive
  // Birth fixes by replacing ρ_target with an independent λ_birth.
  if (cfg_.measurement_driven_birth && !cfg_.adaptive_birth) {
    for (const auto& z : scan) {
      if (!canInitiateTrack(z.model) || !isMeasurementCovariancePsd(z.covariance))
        continue;

      // Smart birth: skip when an existing high-r Bernoulli already
      // gates to this measurement. Walks every hypothesis's Bernoullis
      // and uses the estimator's gate (Mahalanobis for single-Gaussian,
      // any-mode for IMM). The first match wins — cheap and sufficient
      // because the dominant hypothesis carries the highest-r Bernoulli
      // for each id in practice.
      bool claimed_by_existing = false;
      if (cfg_.smart_birth_skip_existing) {
        for (const auto& h : density_.mbm) {
          for (const auto& b : h.bernoullis) {
            if (b.existence_probability < cfg_.smart_birth_skip_r_min) continue;
            Track t = toTrack(b);
            if (estimator_.gate(t, z, cfg_.smart_birth_skip_gate)) {
              claimed_by_existing = true;
              break;
            }
          }
          if (claimed_by_existing) break;
        }
      }
      if (claimed_by_existing) continue;

      // PPP coverage gate. Σ w_c · ℓ(z | c) ≥ threshold ⇒ the
      // existing PPP intensity already covers this measurement;
      // skip the redundant injection that would otherwise inflate
      // the local ρ_target on the next scan.
      if (cfg_.smart_birth_skip_existing_ppp &&
          cfg_.smart_birth_skip_existing_ppp_threshold > 0.0) {
        double coverage = 0.0;
        for (const auto& cprev : density_.ppp) {
          if (cprev.weight <= 0.0) continue;
          Track tprev = toTrack(cprev, current_time_);
          coverage += cprev.weight *
                      std::exp(estimator_.logLikelihood(tprev, z));
          if (coverage >= cfg_.smart_birth_skip_existing_ppp_threshold) break;
        }
        if (coverage >= cfg_.smart_birth_skip_existing_ppp_threshold) continue;
      }

      Track t = estimator_.initiate(z);
      PoissonComponent c;
      c.weight = cfg_.birth_weight_per_measurement;
      fromTrack(c, t);
      density_.ppp.push_back(std::move(c));
    }
  }

  // Per-measurement new-target Bernoulli candidates.
  const auto nts = cfg_.adaptive_birth
                       ? buildAdaptiveBirthCandidates(scan)
                       : buildNewTargetCandidates(scan);

  // PPP undetected-mass decay (§3.3). Uniform p_D for Phase 1.
  for (auto& c : density_.ppp) {
    c.weight *= (1.0 - cfg_.probability_of_detection);
  }

  // Per-parent K-best enumeration.
  std::vector<GlobalHypothesis> children;
  children.reserve(density_.mbm.size() *
                   static_cast<std::size_t>(cfg_.k_best_per_hypothesis));
  // Phase 8 iter 5: clear per-scan birth-id cache. Populated inside
  // enumerateChildren when the cache is active (see below).
  scan_birth_id_cache_.clear();
  int parent_idx_for_cache = 0;
  // Phase 9 review-2 fix: thread parent_idx unconditionally when EITHER
  // adaptive K-best OR cross-parent birth-id cache is on. The original
  // gate (adaptive_k_best only) made `cross_parent_birth_id_cache` a
  // silent no-op when adaptive_k_best was off — docs claimed the flags
  // were independent (mirroring MATLAB's filter-level new-track
  // creation), but the gate broke that contract. The non-adaptive K=1
  // path still enumerates one child per parent and benefits from the
  // cross-parent id share when multiple parents birth the same
  // measurement.
  const bool need_cache = cfg_.adaptive_k_best ||
                          cfg_.cross_parent_birth_id_cache;
  for (const auto& p : density_.mbm) {
    int k_override = -1;
    int parent_idx_arg = -1;  // < 0 disables id-caching (legacy bit-identical)
    if (cfg_.adaptive_k_best) {
      // MATLAB MTT-master: K_p = max(1, ceil(Nhyp_max · w_p)), capped
      // at the per-parent ceiling cfg_.k_best_per_hypothesis. With
      // a single dominant parent (w≈1) the cap binds; with a broad
      // mixture each parent gets ≥ 1. (Phase 8 P2 fix.)
      const double w = std::max(0.0, p.weight);
      const int k_raw = static_cast<int>(std::ceil(
          static_cast<double>(cfg_.max_global_hypotheses) * w));
      k_override = std::clamp(k_raw, 1,
                              std::max(1, cfg_.k_best_per_hypothesis));
    }
    if (need_cache) {
      parent_idx_arg = parent_idx_for_cache;
    }
    const std::size_t children_before = children.size();
    enumerateChildren(p, scan, nts, children, k_override, parent_idx_arg);
    ++parent_idx_for_cache;

    // Phase 9 M2 dominance cutoff (per-parent). When this parent's
    // K-best top child dominates by more than k_best_dominance_log_gap
    // nats, drop the weaker siblings — they would emit phantom-birth
    // Bernoullis into the aggregated output via Σ w·r (Diagnostic A
    // on autoferry_scenario13_anchored). 0 = disabled (legacy
    // behavior, bit-identical).
    if (cfg_.k_best_dominance_log_gap > 0.0 &&
        children.size() > children_before + 1) {
      double top_lw = -std::numeric_limits<double>::infinity();
      for (std::size_t i = children_before; i < children.size(); ++i) {
        if (children[i].log_weight > top_lw) top_lw = children[i].log_weight;
      }
      if (std::isfinite(top_lw)) {
        const double cutoff_lw = top_lw - cfg_.k_best_dominance_log_gap;
        children.erase(
            std::remove_if(
                children.begin() + children_before, children.end(),
                [cutoff_lw](const GlobalHypothesis& h) {
                  return h.log_weight < cutoff_lw;
                }),
            children.end());
      }
    }

    // Phase 9 S3 alt-birth gate (per-parent, lineage-aware). Captures
    // the per-track-hypothesis "born in a weak alt" discriminator the
    // M3 Option A output-side merge probe lacked — without the full
    // structural refactor. Mechanism: when a non-top K-child's log
    // weight is more than alt_birth_log_gap_threshold below the parent's
    // top K-child, strip its NEW-BORN Bernoullis (birth_time ==
    // scan time) while keeping its detection / misdetection
    // contributions. The log_weight of the alt child is unchanged
    // (the assignment is still scored as feasible); only the phantom-
    // birth output mass is suppressed.
    //
    // Different from k_best_dominance_log_gap (drops the whole child)
    // and from min_new_bernoulli_existence (per-cell r_new gate, no
    // sibling context). Active only when ≥ 2 K-children survived
    // (no top-vs-alt comparison otherwise). 0 = disabled (bit-
    // identical to S2 baseline).
    if (cfg_.alt_birth_log_gap_threshold > 0.0 && !scan.empty() &&
        children.size() > children_before + 1) {
      double top_lw = -std::numeric_limits<double>::infinity();
      for (std::size_t i = children_before; i < children.size(); ++i) {
        if (children[i].log_weight > top_lw) top_lw = children[i].log_weight;
      }
      if (std::isfinite(top_lw)) {
        const double cutoff_lw = top_lw - cfg_.alt_birth_log_gap_threshold;
        const double scan_t = scan.front().time.seconds();
        for (std::size_t i = children_before; i < children.size(); ++i) {
          if (children[i].log_weight >= cutoff_lw) continue;
          auto& bs = children[i].bernoullis;
          bs.erase(
              std::remove_if(
                  bs.begin(), bs.end(),
                  [scan_t](const Bernoulli& b) {
                    return b.birth_time.seconds() == scan_t;
                  }),
              bs.end());
        }
      }
    }
  }

  density_.mbm = std::move(children);

  // Task 5 Step-4 fix: apply the deferred window writes once, after every
  // parent has been enumerated against the frozen pre-scan snapshot. This is
  // what makes the surveillance-miss / cooperative-touch windows
  // hypothesis-order-independent.
  mergeStagedActivityMaps();

  // Within-hypothesis Bernoulli merge: fold near-duplicate Bernoullis
  // (Bhattacharyya position-block distance < threshold) into a single
  // Bernoulli keeping the older id.
  if (cfg_.bhattacharyya_merge_threshold > 0.0) {
    for (auto& h : density_.mbm) {
      mergeBernoulliDuplicates(h);
    }
  }

  pruneAndNormalise();

  // Phase 9 S2: keep the per-track view in sync with the flat MBM
  // when the gradual-migration flag is on. The view is a pure
  // re-shape of the post-prune flat representation, so subsequent
  // pipeline stages (refreshAggregatedTracks under the flag) can
  // read it without first re-walking the flat list. No behavioural
  // change — output is still authoritative from the flat path until
  // S3 re-routes it.
  if (cfg_.use_per_track_hypotheses) {
    rebuildPerTrackViewFromFlat(density_);
  }

  // Source-touch history. Walk the highest-weighted hypothesis; for each
  // Bernoulli that CLAIMED a measurement this scan (last_claimed_meas_index
  // >= 0; misdetections carry -1), append a SourceTouch for that exact
  // measurement under the Bernoulli's id. Same window-prune / alive-id-keep
  // semantics as MhtTracker::contribution_history_.
  if (!density_.mbm.empty() && !scan.empty()) {
    const auto& dom = *std::max_element(
        density_.mbm.begin(), density_.mbm.end(),
        [](const GlobalHypothesis& a, const GlobalHypothesis& b) {
          return a.weight < b.weight;
        });
    for (const auto& b : dom.bernoullis) {
      // Attribute a SourceTouch only to the measurement this Bernoulli
      // ACTUALLY claimed this scan. Bernoulli::last_claimed_meas_index (the
      // R2 true-assignment field) is the scan index of the detection/birth
      // that updated it, or -1 for a misdetection (PmbmTracker.cpp:965/1027/
      // 1176). The old "nearest measurement at z.time == b.last_update"
      // heuristic was broken: PmbmTracker::predict advances last_update to the
      // scan time on EVERY Bernoulli (PmbmTypes.hpp:433), detected or not, so
      // a misdetected Bernoulli matched any same-time measurement and was
      // falsely credited the nearest one (no distance bound) — a contribution
      // it never made. That polluted recent_contributions (the emitted
      // SourceTouch provenance) and, under source_aware_misdetection /
      // idle_halflife, fed the miss gate itself.
      const int claimed = b.last_claimed_meas_index;
      if (claimed < 0 || claimed >= static_cast<int>(scan.size())) continue;
      const Measurement* best = &scan[claimed];
      Track::SourceTouch touch;
      touch.sensor = best->sensor;
      touch.source_id = best->source_id;
      touch.vessel_id = best->hints.mmsi;        // per-vessel identity via mmsi
      touch.platform_id = best->hints.platform_id;  // per-vessel identity via platform_id
      touch.time = best->time;
      fillSourceTouchEnu(touch, *best);
      touch.sensor_position_enu = best->sensor_position_enu;
      touch.own_position_std_m = best->sensor_position_std_m;
      touch.covariance_is_default = best->covariance_is_default;
      // §14.11: cumulative output-provenance set, fed from the SAME claimed
      // measurement (dedup, first-seen order) — the MHT tree_sources_ idiom.
      // Not windowed: it is the track's whole-life source set (contributing_sources).
      {
        auto& srcs = bernoulli_sources_[b.id];
        if (std::find(srcs.begin(), srcs.end(), best->source_id) == srcs.end())
          srcs.push_back(best->source_id);
      }
      auto& history = contribution_history_[b.id];
      history.push_back(std::move(touch));
      const std::int64_t window_ns =
          static_cast<std::int64_t>(kContributionWindowSec * 1e9);
      const std::int64_t now_ns = scan.front().time.nanos();
      auto first_keep = std::find_if(
          history.begin(), history.end(),
          [&](const Track::SourceTouch& st) {
            return (now_ns - st.time.nanos()) <= window_ns;
          });
      history.erase(history.begin(), first_keep);
    }
    // Drop history entries whose Bernoulli id is no longer present in
    // any hypothesis (the Bernoulli was pruned).
    std::set<BernoulliId> alive;
    for (const auto& h : density_.mbm) {
      for (const auto& b : h.bernoullis) alive.insert(b.id);
    }
    for (auto it = contribution_history_.begin();
         it != contribution_history_.end();) {
      if (alive.count(it->first) == 0) it = contribution_history_.erase(it);
      else ++it;
    }
    // §14.11: prune the cumulative source set on the same alive-id rule.
    for (auto it = bernoulli_sources_.begin(); it != bernoulli_sources_.end();) {
      if (alive.count(it->first) == 0) it = bernoulli_sources_.erase(it);
      else ++it;
    }
    // Task 5: prune last_activity_check_ alongside contribution_history_.
    for (auto it = last_activity_check_.begin();
         it != last_activity_check_.end();) {
      if (alive.count(it->first) == 0) it = last_activity_check_.erase(it);
      else ++it;
    }
    // Task 6: prune last_cooperative_touch_ alongside the other per-Bernoulli maps.
    for (auto it = last_cooperative_touch_.begin();
         it != last_cooperative_touch_.end();) {
      if (alive.count(it->first) == 0) it = last_cooperative_touch_.erase(it);
      else ++it;
    }
  }

  // Stage 1b: feed the detection model the per-scan outcome, labeled from the
  // dominant post-prune hypothesis — a return claimed by a Bernoulli carries
  // clutter weight 1 − r (its existence), an unclaimed return carries 1.0.
  // Mirrors MhtTracker's producer (MhtTracker.cpp:732-811). Runs AFTER
  // pruneAndNormalise and the cost matrix, so it affects only the NEXT scan's
  // λ_C (no within-scan feedback → deterministic). Gated by feed_clutter_map
  // (default off → never called → bit-identical; Fixed models ignore observe()).
  // Asymmetry (R2 item 5): an empty scan is skipped, so the detection model does
  // not decay its accumulated intensity on scans with no returns. Intentional
  // for now — the learned occupancy is birth-channel-only and a no-return scan
  // carries no positional evidence to feed; revisit if a duty-cycle decay knob
  // is added.
  // The bundle is built when EITHER consumer is wired: the detection model
  // (feed_clutter_map → λ_C adaptation) or the live occupancy sink (birth-only,
  // no λ_C). With neither wired the block never runs → bit-identical to today's.
  const bool feed_detection =
      cfg_.feed_clutter_map && detection_model_ != nullptr;
  const bool feed_occupancy = occupancy_feed_ != nullptr;
  if ((feed_detection || feed_occupancy) && !density_.mbm.empty() &&
      !scan.empty()) {
    const auto& dom = *std::max_element(
        density_.mbm.begin(), density_.mbm.end(),
        [](const GlobalHypothesis& a, const GlobalHypothesis& b) {
          return a.weight < b.weight;
        });
    std::vector<double> claim_r(scan.size(), -1.0);  // < 0 = unclaimed
    // R2: credit each claimed return to the Bernoulli that actually claimed it
    // under this hypothesis's association (index recorded in enumerateChildren),
    // NOT a nearest-neighbour-at-timestamp reconstruction. NN collapsed two
    // close Bernoullis onto one return and fed the other real return as
    // full-weight clutter, driving the dense_clutter lifetime spiral (0.90 →
    // 0.26; eval-log 2026-07-01). The recorded index already respects the
    // (sensor, model) bundle — it IS the assigned measurement — so no separate
    // sensor filter is needed, and it stays correct when a track born on one
    // sensor is updated by another. Bearing-only returns never carry an index
    // (they cannot initiate), so the old sensor_position_enu fallback is gone.
    for (const auto& b : dom.bernoullis) {
      const int j = b.last_claimed_meas_index;
      if (j >= 0 && j < static_cast<int>(scan.size()))
        claim_r[j] = std::max(claim_r[j], b.existence_probability);
    }
    using Key = std::tuple<SensorKind, MeasurementModel>;
    std::map<Key, ISensorDetectionModel::ScanObservation> by_sensor;
    // Coverage-aware decay (increment 6b): capture a representative sensor
    // position per bundle to self-estimate its swept footprint after the loop.
    const bool estimate_cov = cfg_.estimate_coverage_sector;
    std::map<Key, Eigen::Vector2d> cov_sensor;
    // R9 item 1b wiring: collect non-scanning positional anchors (AIS/Cooperative/
    // RemoteTrack) to route to the occupancy VETO — a birth near a known vessel
    // must not be suppressed. Only when an occupancy sink is wired (else empty →
    // no observeVesselFix calls → bit-identical to today). #20: carry an
    // `anchored` flag (AIS nav-status 1=anchor / 5=moored) so the occupancy
    // veto can hold a self-declared stationary vessel for its longer window.
    struct VesselFixIn {
      double t;
      Eigen::Vector2d pos;
      bool anchored;
    };
    std::vector<VesselFixIn> vessel_fixes;
    for (std::size_t j = 0; j < scan.size(); ++j) {
      const Key k{scan[j].sensor, scan[j].model};
      auto it = by_sensor.find(k);
      if (it == by_sensor.end()) {
        ISensorDetectionModel::ScanObservation obs;
        obs.sensor = scan[j].sensor;
        obs.model = scan[j].model;
        obs.num_unassociated = 0;
        obs.time = scan.front().time;
        it = by_sensor.emplace(k, std::move(obs)).first;
      }
      const bool claimed = claim_r[j] >= 0.0;
      const double weight =
          claimed ? std::clamp(1.0 - claim_r[j], 0.0, 1.0) : 1.0;
      if (canInitiateTrack(scan[j].model) && scan[j].value.size() >= 2) {
        it->second.positions.emplace_back(scan[j].value(0), scan[j].value(1));
        // R9 item 1a: exclude non-scanning sources (AIS/Cooperative) from
        // coverage estimation — their returns are vessel positions, not a swept
        // arc, so a self-estimated wedge would over-claim coverage (unsafe decay).
        if (estimate_cov && !isNonScanningSource(scan[j].sensor) &&
            cov_sensor.find(k) == cov_sensor.end())
          cov_sensor[k] = scan[j].sensor_position_enu;
        // Same non-scanning predicate, opposite use: a vessel POSITION from
        // AIS/Cooperative/RemoteTrack is a corroboration fix for the birth veto.
        if (feed_occupancy && isNonScanningSource(scan[j].sensor)) {
          // Translate AIS nav-status → the kind-agnostic "anchored" flag the
          // occupancy port speaks (1 = at anchor, 5 = moored).
          const bool anchored =
              scan[j].hints.nav_status.has_value() &&
              (*scan[j].hints.nav_status == 1 || *scan[j].hints.nav_status == 5);
          vessel_fixes.push_back(
              {scan[j].time.seconds(),
               Eigen::Vector2d(scan[j].value(0), scan[j].value(1)), anchored});
        }
        if (weight > 0.0) {
          it->second.clutter_positions.emplace_back(scan[j].value(0),
                                                    scan[j].value(1));
          it->second.clutter_position_weights.push_back(weight);
        }
      }
      if (scan[j].model == MeasurementModel::Bearing2D &&
          scan[j].value.size() >= 1) {
        it->second.bearings.push_back(scan[j].value(0));
        if (weight > 0.0) {
          it->second.clutter_bearings.push_back(scan[j].value(0));
          it->second.clutter_bearing_weights.push_back(weight);
        }
      }
      if (!claimed) ++it->second.num_unassociated;
    }
    // Self-estimate each bundle's swept footprint from its position returns
    // (about the representative sensor position). Under-estimated coverage is the
    // safe direction (unobserved cells simply don't decay). Bundles with no
    // positional returns (bearing-only) get no footprint → full coverage.
    if (estimate_cov) {
      for (auto& kv : by_sensor) {
        const auto sit = cov_sensor.find(kv.first);
        if (sit == cov_sensor.end()) continue;
        kv.second.coverage = ISensorDetectionModel::CoverageSector::fromReturns(
            sit->second, kv.second.positions, cfg_.coverage_az_pad_rad,
            cfg_.coverage_range_pad_frac, cfg_.coverage_cluster_gap_rad);
      }
    }
    std::vector<ISensorDetectionModel::ScanObservation> bundle;
    bundle.reserve(by_sensor.size());
    for (auto& kv : by_sensor) bundle.push_back(std::move(kv.second));
    if (feed_detection) detection_model_->observe(bundle);
    if (feed_occupancy) {
      // Route corroboration fixes BEFORE the structure feed, so this scan's
      // known-vessel anchors are present for next scan's birth-veto queries
      // (same recency as the structure update; observe() prunes stale fixes).
      for (const auto& vf : vessel_fixes)
        occupancy_feed_->observeVesselFix(vf.t, vf.pos, vf.anchored);
      occupancy_feed_->observe(bundle);
    }
  }

  aggregated_tracks_dirty_ = true;
  // Task 6: fire stale signals for cooperative-overdue tracks.
  if (stale_sink_ != nullptr && !cooperative_overdue_ids_.empty()) {
    const auto& ts = tracks();  // triggers refreshAggregatedTracks (Coasting)
    for (const auto& t : ts) {
      if (cooperative_overdue_ids_.count(t.id.value)) {
        stale_sink_->onTrackStale(t.id, current_time_);
      }
    }
  }
  // Phase 4(B) push-based lifecycle events. Diffs against the prior-
  // scan emitted set. No-op when no sink is wired.
  if (track_sink_ != nullptr && !scan.empty()) {
    firePmbmLifecycleEvents(scan.front().time);
  }
  // Backlog #25 diagnostic emission (no-op unless a diagnostic sink is set).
  // Runs AFTER pruneAndNormalise + lifecycle so it observes the exact
  // post-scan density the states.csv export sees.
  if (diag_sink_ != nullptr) {
    emitPmbmDiagnostics(current_time_);
    ++diag_scan_counter_;
  }
}

namespace {

// Bhattacharyya distance between two 2-D Gaussians on the position
// block (first two state components). Standard form:
//   d_B = 1/8 · (m1-m2)' P^-1 (m1-m2) + 1/2 · ln(det(P) / sqrt(det(P1)·det(P2)))
// with P = (P1 + P2) / 2. Returns +∞ for degenerate covariances.
// Bhattacharyya distance on the (px, py, vx, vy) block when both states
// have ≥ 4 dims, falling back to (px, py) when one is shorter. Position-
// only would merge two Bernoullis sitting on top of each other with
// opposite velocities — an id-merge bomb on crossings; velocity-aware
// distance keeps them separate (Phase 8 R6 fix).
double bhattacharyyaState(const Eigen::VectorXd& mean_a,
                          const Eigen::MatrixXd& cov_a,
                          const Eigen::VectorXd& mean_b,
                          const Eigen::MatrixXd& cov_b) {
  if (mean_a.size() < 2 || mean_b.size() < 2) return kInf;
  if (cov_a.rows() < 2 || cov_b.rows() < 2) return kInf;
  const int dim = (mean_a.size() >= 4 && mean_b.size() >= 4 &&
                   cov_a.rows() >= 4 && cov_b.rows() >= 4) ? 4 : 2;
  const Eigen::VectorXd d = mean_a.head(dim) - mean_b.head(dim);
  const Eigen::MatrixXd Pa = cov_a.topLeftCorner(dim, dim);
  const Eigen::MatrixXd Pb = cov_b.topLeftCorner(dim, dim);
  const Eigen::MatrixXd P = 0.5 * (Pa + Pb);
  // LDLT-based solves are PSD-stable; determinant via the LDLT factor
  // avoids the ill-conditioning of the naive det() on large covariances.
  Eigen::LDLT<Eigen::MatrixXd> ldlt_P(P);
  Eigen::LDLT<Eigen::MatrixXd> ldlt_Pa(Pa);
  Eigen::LDLT<Eigen::MatrixXd> ldlt_Pb(Pb);
  if (ldlt_P.info() != Eigen::Success ||
      ldlt_Pa.info() != Eigen::Success ||
      ldlt_Pb.info() != Eigen::Success) return kInf;
  const Eigen::VectorXd diag_P = ldlt_P.vectorD();
  const Eigen::VectorXd diag_Pa = ldlt_Pa.vectorD();
  const Eigen::VectorXd diag_Pb = ldlt_Pb.vectorD();
  if ((diag_P.array() <= 0.0).any() ||
      (diag_Pa.array() <= 0.0).any() ||
      (diag_Pb.array() <= 0.0).any()) return kInf;
  const double log_det_P  = diag_P.array().log().sum();
  const double log_det_Pa = diag_Pa.array().log().sum();
  const double log_det_Pb = diag_Pb.array().log().sum();
  const double mahal = d.transpose() * ldlt_P.solve(d);
  return 0.125 * mahal + 0.5 * (log_det_P - 0.5 * (log_det_Pa + log_det_Pb));
}

}  // namespace

void PmbmTracker::mergeBernoulliDuplicates(GlobalHypothesis& h) const {
  // Sort by id ascending so the survivor (kept) carries the older id.
  std::sort(h.bernoullis.begin(), h.bernoullis.end(),
            [](const Bernoulli& a, const Bernoulli& b) {
              return a.id < b.id;
            });
  std::vector<bool> dead(h.bernoullis.size(), false);
  for (std::size_t i = 0; i < h.bernoullis.size(); ++i) {
    if (dead[i]) continue;
    for (std::size_t j = i + 1; j < h.bernoullis.size(); ++j) {
      if (dead[j]) continue;
      const double dB = bhattacharyyaState(
          h.bernoullis[i].mean, h.bernoullis[i].covariance,
          h.bernoullis[j].mean, h.bernoullis[j].covariance);
      if (dB > cfg_.bhattacharyya_merge_threshold) continue;

      // NOTE (review, distinct same-scan claims): a state-close pair that each
      // claimed a DIFFERENT measurement this scan is NOT necessarily two targets
      // — a fused batch is multi-sensor, so one physical target detected by two
      // sensors (or one large target split into two radar plots) legitimately
      // produces exactly that. A guard that refused such merges would leave a
      // duplicate track per multi-sensor vessel, so we do NOT guard here. The
      // side effect is a known limitation of the single-index clutter feed: when
      // two claimed Bernoullis fold, only the survivor's claim (below) is
      // credited and the folded measurement is fed as clutter. Feeding it as
      // clutter is defensible (the merged target explains one return); the
      // proper fix, when feed_clutter_map graduates from experimental, is to
      // carry a claim SET through the merge rather than a single index.

      // Merge j INTO i. r_merged = max(r_i, r_j) — the merge gate fires
      // only when (px, py, vx, vy) overlap closely, which in practice
      // means the two Bernoullis trace back to the same physical
      // target (split→rejoin under measurement ambiguity). The
      // textbook independent-existence fold r_m = 1 - (1-r_i)(1-r_j)
      // assumes uncorrelated existence, which double-counts when
      // duplicates share a parent. Phase 8 R1 fix: keep the
      // best-supported existence rather than inflate.
      // Mean / cov: r-weighted moment match.
      Bernoulli& a = h.bernoullis[i];
      const Bernoulli& b = h.bernoullis[j];
      const double r_a = a.existence_probability;
      const double r_b = b.existence_probability;
      const double r_m = std::max(r_a, r_b);
      const double wa = r_a, wb = r_b;
      const double ws = wa + wb;
      if (ws > 0.0 && a.mean.size() == b.mean.size()) {
        const Eigen::VectorXd new_mean = (wa * a.mean + wb * b.mean) / ws;
        const Eigen::VectorXd d_a = a.mean - new_mean;
        const Eigen::VectorXd d_b = b.mean - new_mean;
        Eigen::MatrixXd new_cov =
            (wa * (a.covariance + d_a * d_a.transpose()) +
             wb * (b.covariance + d_b * d_b.transpose())) / ws;
        a.mean = new_mean;
        a.covariance = std::move(new_cov);
      }
      a.existence_probability = r_m;
      // Clutter-feed claim carry: the survivor keeps a REAL claim over none, and
      // when both are real follows the higher-existence duplicate (consistent
      // with r_m = max). Guarded so it can never drop a's real claim in favour of
      // b's -1 (the r_b > r_a branch alone would, when b is a misdetection with
      // higher existence than a — not saved by any incidental invariant here).
      const bool take_b_claim =
          b.last_claimed_meas_index >= 0 &&
          (a.last_claimed_meas_index < 0 || r_b > r_a);
      if (take_b_claim) {
        a.last_claimed_meas_index = b.last_claimed_meas_index;
        // #25 P2b: keep the innovation attached to the claim it belongs to.
        if (diag_sink_ != nullptr) {
          a.last_innovation_enu = b.last_innovation_enu;
          a.last_innovation_norm_m = b.last_innovation_norm_m;
        }
      }
      // R11: the two folding Bernoullis are the same logical target, so the
      // survivor keeps any identity it already has and inherits the duplicate's
      // when it had none (never drop a real id in favour of a nullopt).
      if (!a.mmsi.has_value()) a.mmsi = b.mmsi;
      if (!a.platform_id.has_value()) a.platform_id = b.platform_id;
      if (!a.heading_deg.has_value()) a.heading_deg = b.heading_deg;
      if (!a.nav_status.has_value()) a.nav_status = b.nav_status;
      if (b.last_update.seconds() > a.last_update.seconds())
        a.last_update = b.last_update;
      dead[j] = true;
    }
  }
  // Compact.
  std::vector<Bernoulli> kept;
  kept.reserve(h.bernoullis.size());
  for (std::size_t k = 0; k < h.bernoullis.size(); ++k) {
    if (!dead[k]) kept.push_back(std::move(h.bernoullis[k]));
  }
  h.bernoullis = std::move(kept);
}

const std::vector<Track>& PmbmTracker::tracks() const {
  if (aggregated_tracks_dirty_) {
    refreshAggregatedTracks();
    aggregated_tracks_dirty_ = false;
  }
  return aggregated_tracks_;
}

void PmbmTracker::firePmbmLifecycleEvents(Timestamp event_time) {
  if (track_sink_ == nullptr) return;
  // Compute current emitted track set + statuses by id.
  std::map<std::uint64_t, TrackStatus> current;
  const auto& ts = tracks();
  for (const auto& t : ts) current[t.id.value] = t.status;

  // Initiated (new ids, status Tentative) and Confirmed (new ids
  // status Confirmed, or prior Tentative → Confirmed transition).
  for (const auto& [id, status] : current) {
    const TrackId tid{id};
    const TrackLifecycleEvent e{tid, event_time, status};
    auto pit = prev_emitted_statuses_.find(id);
    if (pit == prev_emitted_statuses_.end()) {
      // Newly emitted this scan.
      if (status == TrackStatus::Confirmed) {
        track_sink_->onTrackInitiated(e);  // existence preceded
                                            // confirmation in one scan
        track_sink_->onTrackConfirmed(e);
      } else {
        track_sink_->onTrackInitiated(e);
      }
    } else {
      // Existed last scan; check status transition.
      if (pit->second != TrackStatus::Confirmed &&
          status == TrackStatus::Confirmed) {
        track_sink_->onTrackConfirmed(e);
      }
    }
    // Every track present this scan emits onTrackUpdated.
    track_sink_->onTrackUpdated(e);
  }

  // Deleted (prior id absent now). Emit BEFORE updating the snapshot
  // so trajectoryFor(id) inside the handler hits the prior-scan
  // trajectory snapshot. Status reported is the pre-deletion status.
  for (const auto& [id, status] : prev_emitted_statuses_) {
    if (current.find(id) != current.end()) continue;
    const TrackId tid{id};
    track_sink_->onTrackDeleted({tid, event_time, status});
  }

  prev_emitted_statuses_ = std::move(current);
  // Refresh the trajectory snapshot from the current dominant
  // hypothesis. Done AFTER the diff so the handler above still saw
  // the prior-scan trajectory for deleted ids.
  prev_emitted_trajectories_.clear();
  for (const auto& t : ts) {
    auto traj = trajectoryFor(t.id.value);
    if (!traj.empty()) prev_emitted_trajectories_[t.id.value] =
        std::move(traj);
  }
}

std::map<BernoulliId, std::vector<TrajectoryPoint>>
PmbmTracker::collectSmoothedTrajectories() const {
  std::map<BernoulliId, std::vector<TrajectoryPoint>> out;
  if (cfg_.trajectory_window_scans == 0) return out;
  if (density_.mbm.empty()) return out;
  // Pick the dominant hypothesis (highest weight).
  const auto& dom = *std::max_element(
      density_.mbm.begin(), density_.mbm.end(),
      [](const GlobalHypothesis& a, const GlobalHypothesis& b) {
        return a.weight < b.weight;
      });
  for (const auto& b : dom.bernoullis) {
    if (b.trajectory.empty()) continue;
    auto traj = b.trajectory;
    rtsSmoothTrajectory(traj);
    out.emplace(b.id, std::move(traj));
  }
  return out;
}

std::vector<TrajectoryPoint> PmbmTracker::trajectoryFor(BernoulliId id) const {
  // Pick the highest-weight hypothesis containing this id. Trajectory
  // is per-Bernoulli (per-hypothesis), so the dominant interpretation
  // is the right one to expose (matches refreshAggregatedTracks'
  // emission of the dominant-weighted state).
  const GlobalHypothesis* best_h = nullptr;
  const Bernoulli* best_b = nullptr;
  for (const auto& h : density_.mbm) {
    for (const auto& b : h.bernoullis) {
      if (b.id != id) continue;
      if (best_h == nullptr || h.weight > best_h->weight) {
        best_h = &h;
        best_b = &b;
      }
    }
  }
  if (best_b != nullptr) return best_b->trajectory;
  // Phase 4(D) fallback: the id is no longer live (e.g. pruned this
  // scan). Inside an onTrackDeleted handler, return the prior-scan
  // trajectory snapshot so consumers can drain the final history.
  auto it = prev_emitted_trajectories_.find(id);
  if (it != prev_emitted_trajectories_.end()) return it->second;
  return {};
}

void PmbmTracker::refreshAggregatedTracks() const {
  // Aggregate (w^j · r^{j,id}) across hypotheses for each Bernoulli id.
  // std::map ordered by id keeps the emission order deterministic across
  // runs — critical for the determinism contract and for diff-friendly
  // bench output.
  struct Acc {
    double mass{0.0};
    Eigen::VectorXd mean_acc;       // Σ w·r · μ
    Eigen::MatrixXd cov_acc;        // Σ w·r · (P + μμ')
    Timestamp last_update{};
    // M3 iter-6 discriminator signals — populated unconditionally
    // (cheap), consumed only when the corresponding output-merge
    // gate knob is enabled.
    double earliest_birth_sec{std::numeric_limits<double>::infinity()};
    int hyp_count{0};
    // R11 identity surfacing: the vessel identity carried by the contributing
    // Bernoullis. Taken from the highest-mass hypothesis that actually HAS an
    // identity (a dominant hypothesis with no id must not shadow an id carried
    // by a slightly-weaker one). `*_id_mass` is that contributor's mass, kept
    // so the cross-id merge below can prefer the stronger source.
    std::optional<std::uint32_t> mmsi;
    double mmsi_mass{0.0};
    std::optional<std::uint64_t> platform_id;
    double platform_id_mass{0.0};
    std::optional<double> heading_deg;
    double heading_deg_mass{0.0};
    std::optional<std::uint8_t> nav_status;
    double nav_status_mass{0.0};
  };
  std::map<BernoulliId, Acc> by_id;

  for (const auto& h : density_.mbm) {
    for (const auto& b : h.bernoullis) {
      const double m = h.weight * b.existence_probability;
      if (m <= 0.0) continue;
      Acc& a = by_id[b.id];
      if (a.mean_acc.size() == 0) {
        a.mean_acc = Eigen::VectorXd::Zero(b.mean.size());
        a.cov_acc = Eigen::MatrixXd::Zero(b.mean.size(), b.mean.size());
      }
      a.mass += m;
      a.mean_acc += m * b.mean;
      a.cov_acc += m * (b.covariance + b.mean * b.mean.transpose());
      if (b.last_update.seconds() > a.last_update.seconds()) {
        a.last_update = b.last_update;
      }
      const double birth_s = b.birth_time.seconds();
      if (birth_s < a.earliest_birth_sec) a.earliest_birth_sec = birth_s;
      a.hyp_count += 1;
      // R11: adopt each id from the strongest hypothesis that carries it.
      if (b.mmsi.has_value() && m > a.mmsi_mass) {
        a.mmsi = b.mmsi;
        a.mmsi_mass = m;
      }
      if (b.platform_id.has_value() && m > a.platform_id_mass) {
        a.platform_id = b.platform_id;
        a.platform_id_mass = m;
      }
      if (b.heading_deg.has_value() && m > a.heading_deg_mass) {
        a.heading_deg = b.heading_deg;
        a.heading_deg_mass = m;
      }
      if (b.nav_status.has_value() && m > a.nav_status_mass) {
        a.nav_status = b.nav_status;
        a.nav_status_mass = m;
      }
    }
  }

  // Phase 9 M3 Option A: output-side cross-id birth merge. Walks
  // pairs of aggregated ids, folds spatially-close pairs (Bhattacharyya
  // distance below threshold) into the older id (id-stability
  // invariant — std::map<BernoulliId, ...> orders ascending). Fixes
  // the K=3 phantom-birth leak from Diagnostic A: alt hypotheses
  // birth fresh ids that don't merge in the within-hypothesis pass
  // (different hyp, different id) but sit on top of the same
  // physical target — the output aggregation is the only layer that
  // can see across ids. ≤ 0 disables (bit-identical to legacy).
  if (cfg_.output_merge_bhattacharyya_threshold > 0.0 && by_id.size() > 1) {
    std::vector<BernoulliId> ids;
    ids.reserve(by_id.size());
    for (const auto& kv : by_id) ids.push_back(kv.first);
    // Snapshot normalised (mean, cov) per surviving id; recompute
    // when an id absorbs a sibling so the second-pass comparison
    // sees the merged state, not the stale one.
    std::map<BernoulliId, std::pair<Eigen::VectorXd, Eigen::MatrixXd>> stats;
    auto computeStats = [&](const Acc& a) {
      Eigen::VectorXd mean = a.mean_acc / a.mass;
      Eigen::MatrixXd cov = (a.cov_acc / a.mass) - mean * mean.transpose();
      return std::make_pair(std::move(mean), std::move(cov));
    };
    for (BernoulliId id : ids) {
      const Acc& a = by_id.at(id);
      if (a.mass > 0.0) stats[id] = computeStats(a);
    }
    for (std::size_t i = 0; i < ids.size(); ++i) {
      auto ait = by_id.find(ids[i]);
      if (ait == by_id.end() || ait->second.mass <= 0.0) continue;
      for (std::size_t j = i + 1; j < ids.size(); ++j) {
        auto bit = by_id.find(ids[j]);
        if (bit == by_id.end() || bit->second.mass <= 0.0) continue;
        const auto& sa = stats[ids[i]];
        const auto& sb = stats[ids[j]];
        if (sa.first.size() != sb.first.size()) continue;
        const double dB = bhattacharyyaState(sa.first, sa.second,
                                             sb.first, sb.second);
        if (dB > cfg_.output_merge_bhattacharyya_threshold) continue;
        // M3 iter-6 age gate: both ids must be "young" (born within
        // the last output_merge_max_age_sec seconds). 0 = no gate.
        if (cfg_.output_merge_max_age_sec > 0.0) {
          const double now = current_time_.seconds();
          const double age_a = now - ait->second.earliest_birth_sec;
          const double age_b = now - bit->second.earliest_birth_sec;
          if (std::max(age_a, age_b) > cfg_.output_merge_max_age_sec) continue;
        }
        // M3 iter-6 hyp-count gate: at least one of the two ids must
        // be "weakly supported" (in fewer than max_hyp_support
        // hypotheses). 0 = no gate.
        if (cfg_.output_merge_max_hyp_support > 0 &&
            std::min(ait->second.hyp_count, bit->second.hyp_count) >=
                cfg_.output_merge_max_hyp_support) {
          continue;
        }
        // Fold j INTO i (ids[i] < ids[j] by std::map ordering, so
        // i is the older id — survives, id-stability invariant).
        Acc& a = ait->second;
        Acc& b = bit->second;
        a.mass += b.mass;
        a.mean_acc += b.mean_acc;
        a.cov_acc += b.cov_acc;
        if (b.last_update.seconds() > a.last_update.seconds()) {
          a.last_update = b.last_update;
        }
        // Fold discriminator fields too so further j-pair compares
        // see merged state.
        if (b.earliest_birth_sec < a.earliest_birth_sec) {
          a.earliest_birth_sec = b.earliest_birth_sec;
        }
        a.hyp_count += b.hyp_count;
        // R11: the folded id may carry the identity the survivor lacks (or a
        // stronger source of it) — same mass-preference as the accumulate loop.
        if (b.mmsi.has_value() && b.mmsi_mass > a.mmsi_mass) {
          a.mmsi = b.mmsi;
          a.mmsi_mass = b.mmsi_mass;
        }
        if (b.platform_id.has_value() && b.platform_id_mass > a.platform_id_mass) {
          a.platform_id = b.platform_id;
          a.platform_id_mass = b.platform_id_mass;
        }
        if (b.heading_deg.has_value() && b.heading_deg_mass > a.heading_deg_mass) {
          a.heading_deg = b.heading_deg;
          a.heading_deg_mass = b.heading_deg_mass;
        }
        if (b.nav_status.has_value() && b.nav_status_mass > a.nav_status_mass) {
          a.nav_status = b.nav_status;
          a.nav_status_mass = b.nav_status_mass;
        }
        by_id.erase(bit);
        // Recompute i's snapshot for any further j-pair comparisons.
        stats[ids[i]] = computeStats(a);
      }
    }
  }

  aggregated_tracks_.clear();
  aggregated_tracks_.reserve(by_id.size());
  for (const auto& [id, a] : by_id) {
    if (a.mass < cfg_.output_existence_floor) continue;
    Track t;
    t.id.value = id;
    t.last_update = a.last_update;
    t.existence_probability = std::min(a.mass, 1.0);
    t.status = (a.mass >= cfg_.confirm_threshold) ? TrackStatus::Confirmed
                                                   : TrackStatus::Tentative;
    // Task 6: cooperative own-identity overdue → override with Coasting.
    // Existence is NOT reduced (spec §9c: "comms loss ≠ vessel sank").
    if (cooperative_overdue_ids_.count(id)) {
      t.status = TrackStatus::Coasting;
    }
    t.state = a.mean_acc / a.mass;
    // Moment-matched covariance: E[P + μμ'] − mean·mean'
    t.covariance = (a.cov_acc / a.mass) - (t.state * t.state.transpose());
    // R11 identity surfacing: mmsi (AIS) + platform_id (cooperative/remote).
    // #20: target-reported heading + AIS nav-status on the same footing.
    t.attributes.mmsi = a.mmsi;
    t.attributes.platform_id = a.platform_id;
    t.attributes.heading_deg = a.heading_deg;
    t.attributes.nav_status = a.nav_status;
    auto hit = contribution_history_.find(id);
    if (hit != contribution_history_.end()) {
      t.recent_contributions = hit->second;
    }
    // §14.11: surface the cumulative genuine-contributor set on the output field,
    // matching the flat/MHT contributing_sources semantics (ordered, deduplicated).
    auto src_hit = bernoulli_sources_.find(id);
    if (src_hit != bernoulli_sources_.end()) {
      t.contributing_sources = src_hit->second;
    }
    aggregated_tracks_.push_back(std::move(t));
  }
}

void PmbmTracker::applyInnovationGate(Bernoulli& b) const {
  // Kinematic-only treatment of the velocity (indices 2,3) and, when the state
  // carries it, the turn-rate (index 4). ACCEPT the position (indices 0,1 are
  // left as the measurement pulled them); TREAT the untrustworthy velocity so
  // the runaway cannot propagate. Applied to the moment-matched mean/covariance
  // AND to every IMM mode — the IMM predict reads imm_means/imm_covariances, so
  // fixing only the moment-matched projection would be re-corrupted next scan.
  const double floor = cfg_.innov_gate_velocity_var_floor;
  const bool reset =
      cfg_.innov_gate_action == Config::InnovationGateAction::kVelocityReset;
  const int kIdx[3] = {2, 3, 4};

  auto treat_mean = [&](Eigen::VectorXd& mean) {
    if (!reset) return;  // deweight keeps the mean; only reset zeroes it
    for (int idx : kIdx)
      if (idx < mean.size()) mean(idx) = 0.0;
  };
  auto treat_cov = [&](Eigen::MatrixXd& cov) {
    for (int idx : kIdx) {
      if (idx >= cov.rows() || idx >= cov.cols()) continue;
      // Raise (never shrink) the marginal variance to the prior floor and clear
      // this component's cross-covariances, so the widened velocity/turn-rate
      // uncertainty is not immediately re-tightened through position on the
      // next update — the filter re-learns velocity from subsequent positions.
      if (cov(idx, idx) < floor) {
        for (int j = 0; j < cov.rows(); ++j) { cov(idx, j) = 0.0; cov(j, idx) = 0.0; }
        cov(idx, idx) = floor;
      }
    }
  };

  treat_mean(b.mean);
  treat_cov(b.covariance);
  for (int k = 0; k < b.imm_means.cols(); ++k) {
    if (reset)
      for (int idx : kIdx)
        if (idx < b.imm_means.rows()) b.imm_means(idx, k) = 0.0;
  }
  for (auto& c : b.imm_covariances) treat_cov(c);
}

void PmbmTracker::emitPmbmDiagnostics(Timestamp t) {
  // Diagnostic-only (backlog #25). Never mutates tracking state. Mirrors the
  // refreshAggregatedTracks per-id aggregation (Σ_j w^j·r^{j,id}) but keeps
  // EVERY id — including those below output_existence_floor — so the r/mass
  // trajectory of a dying track is observable before and through its loss. The
  // dominant-hypothesis Bernoulli supplies the mechanism detail (r, the claimed
  // measurement index = detection vs miss, and the state speed = divergence).
  // The output_merge cross-id fold is intentionally NOT applied here (it is
  // disabled — threshold 0 — on imm_cv_ct_pmbm_coverage_land, so this matches
  // the emitted output exactly for that config).
  PmbmScanDiag d;
  d.scan_index = diag_scan_counter_;
  d.time_s = t.seconds();
  d.n_measurements = diag_last_scan_meas_count_;
  d.n_global_hypotheses = static_cast<int>(density_.mbm.size());
  d.n_hyp_dropped_floor = diag_hyp_dropped_floor_;
  d.n_hyp_dropped_cap = diag_hyp_dropped_cap_;
  d.n_bernoulli_pruned_rmin = diag_bernoulli_pruned_rmin_;

  struct Agg {
    double mass{0.0};
    int hyp_count{0};
    Eigen::VectorXd mean_acc;
  };
  std::map<BernoulliId, Agg> by_id;
  int n_bern_total = 0;
  for (const auto& h : density_.mbm) {
    for (const auto& b : h.bernoullis) {
      ++n_bern_total;
      const double m = h.weight * b.existence_probability;
      if (m <= 0.0) continue;
      Agg& a = by_id[b.id];
      if (a.mean_acc.size() == 0)
        a.mean_acc = Eigen::VectorXd::Zero(b.mean.size());
      a.mass += m;
      a.hyp_count += 1;
      a.mean_acc += m * b.mean;
    }
  }
  d.n_bernoulli_total = n_bern_total;
  d.n_ids = static_cast<int>(by_id.size());

  // Dominant (max-weight) global hypothesis: per-id mechanism detail.
  const GlobalHypothesis* dom = nullptr;
  for (const auto& h : density_.mbm) {
    if (dom == nullptr || h.weight > dom->weight) dom = &h;
  }
  std::map<BernoulliId, const Bernoulli*> dom_b;
  if (dom != nullptr) {
    for (const auto& b : dom->bernoullis) dom_b[b.id] = &b;
  }

  d.bernoullis.reserve(by_id.size());
  for (const auto& [id, a] : by_id) {
    PmbmBernoulliDiag bd;
    bd.id = id;
    bd.agg_mass = a.mass;
    bd.hyp_count = a.hyp_count;
    const Eigen::VectorXd mean =
        (a.mass > 0.0) ? (a.mean_acc / a.mass).eval() : a.mean_acc;
    if (mean.size() > 0) bd.east_m = mean(0);
    if (mean.size() > 1) bd.north_m = mean(1);
    auto it = dom_b.find(id);
    if (it != dom_b.end()) {
      const Bernoulli* b = it->second;
      bd.existence_r_best = b->existence_probability;
      bd.claimed_meas_index = b->last_claimed_meas_index;
      bd.speed_mps = (b->mean.size() >= 4)
                         ? std::sqrt(b->mean(2) * b->mean(2) +
                                     b->mean(3) * b->mean(3))
                         : -1.0;
      bd.in_dominant = true;
      // #25 Phase 2b: true applied-measurement position innovation + IMM weights
      // of the dominant-hyp Bernoulli (sentinel norm −1 when it misdetected /
      // was born this scan; imm empty when single-Gaussian).
      bd.innov_east_m = b->last_innovation_enu.x();
      bd.innov_north_m = b->last_innovation_enu.y();
      bd.innov_norm_m = b->last_innovation_norm_m;
      if (b->imm_mode_probabilities.size() > 0)
        bd.imm_mode_weights.assign(
            b->imm_mode_probabilities.data(),
            b->imm_mode_probabilities.data() +
                b->imm_mode_probabilities.size());
    } else {
      bd.existence_r_best = 0.0;
      bd.claimed_meas_index = -2;  // absent from the dominant hypothesis
      bd.speed_mps = -1.0;
      bd.in_dominant = false;
    }
    bd.in_output = a.mass >= cfg_.output_existence_floor;
    bd.confirmed = a.mass >= cfg_.confirm_threshold;
    d.bernoullis.push_back(std::move(bd));
  }

  diag_sink_->onPmbmScan(d);
}

}  // namespace navtracker::pmbm
