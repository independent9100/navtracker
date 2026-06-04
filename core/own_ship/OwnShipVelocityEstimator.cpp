#include "core/own_ship/OwnShipVelocityEstimator.hpp"

#include <cmath>

namespace navtracker {
namespace {

// Local copy of the LS fit helper used by UereEstimator. Kept private here
// to keep the two estimators independently buildable; if a third consumer
// appears, lift into a shared core/own_ship/LinearFit.hpp.
struct LinearFit {
  double a{0.0};      // intercept
  double b{0.0};      // slope (velocity component)
  double ss_res{0.0}; // sum residual^2
  double sxx{0.0};    // sum (dt - mean_dt)^2 — needed for slope std error
};

template <class Getter>
LinearFit fitAxis(const std::deque<OwnShipVelocityEstimator::Sample>& samples,
                  double t0,
                  Getter get) {
  const double n = static_cast<double>(samples.size());
  double sum_dt = 0.0, sum_p = 0.0;
  for (const auto& s : samples) {
    sum_dt += (s.t - t0);
    sum_p  += get(s);
  }
  const double mean_dt = sum_dt / n;
  const double mean_p  = sum_p  / n;
  double sxy = 0.0, sxx = 0.0;
  for (const auto& s : samples) {
    const double dt = (s.t - t0) - mean_dt;
    const double dp = get(s) - mean_p;
    sxy += dt * dp;
    sxx += dt * dt;
  }
  LinearFit f;
  f.b = sxx > 0.0 ? sxy / sxx : 0.0;
  f.a = mean_p - f.b * mean_dt;
  f.sxx = sxx;
  double ss = 0.0;
  for (const auto& s : samples) {
    const double dt = s.t - t0;
    const double r  = get(s) - (f.a + f.b * dt);
    ss += r * r;
  }
  f.ss_res = ss;
  return f;
}

}  // namespace

OwnShipVelocityEstimator::OwnShipVelocityEstimator(
    OwnShipVelocityEstimatorConfig cfg) : cfg_(cfg) {}

void OwnShipVelocityEstimator::observe(Timestamp t,
                                       double x_enu,
                                       double y_enu) {
  samples_.push_back({static_cast<double>(t.nanos()) * 1e-9, x_enu, y_enu});
  while (samples_.size() > cfg_.window_size) samples_.pop_front();
}

OwnShipVelocityEstimate OwnShipVelocityEstimator::current() const {
  OwnShipVelocityEstimate est;
  if (samples_.size() < cfg_.window_size) return est;
  if (cfg_.window_size < 4) return est;  // need at least 2 samples per half

  const double t0 = samples_.front().t;

  const auto fit_x = fitAxis(samples_, t0,
                             [](const Sample& s) { return s.x; });
  const auto fit_y = fitAxis(samples_, t0,
                             [](const Sample& s) { return s.y; });

  const double n = static_cast<double>(samples_.size());
  if (n <= 2.0) return est;
  const double var = (fit_x.ss_res + fit_y.ss_res) / (2.0 * (n - 2.0));
  if (!std::isfinite(var) || var < 0.0) return est;
  const double sigma_pos = std::sqrt(var);

  // Per-axis slope standard error: sigma_pos / sqrt(sum (dt_i - mean_dt)^2).
  // sxx is identical for both axes (same dt samples), but keep both for
  // symmetry with the fit results.
  const double sigma_v_x = fit_x.sxx > 0.0
                               ? sigma_pos / std::sqrt(fit_x.sxx)
                               : 0.0;
  const double sigma_v_y = fit_y.sxx > 0.0
                               ? sigma_pos / std::sqrt(fit_y.sxx)
                               : 0.0;
  double sigma_v = 0.5 * (sigma_v_x + sigma_v_y);

  // Maneuver gating: same noise-aware two-halves rule as UereEstimator.
  // Split window in halves, fit velocity in each, reject if the magnitude
  // of the half-to-half velocity difference exceeds the threshold
  // augmented by 3*sqrt(2)*sigma_v_half (the 3-sigma envelope on the
  // velocity-difference noise from finite-window LS).
  const std::size_t half = samples_.size() / 2;
  std::deque<Sample> h1(samples_.begin(), samples_.begin() + half);
  std::deque<Sample> h2(samples_.begin() + half, samples_.end());
  if (h1.size() >= 2 && h2.size() >= 2) {
    const auto fx1 = fitAxis(h1, h1.front().t,
                             [](const Sample& s) { return s.x; });
    const auto fx2 = fitAxis(h2, h2.front().t,
                             [](const Sample& s) { return s.x; });
    const auto fy1 = fitAxis(h1, h1.front().t,
                             [](const Sample& s) { return s.y; });
    const auto fy2 = fitAxis(h2, h2.front().t,
                             [](const Sample& s) { return s.y; });
    const double dvx = fx2.b - fx1.b;
    const double dvy = fy2.b - fy1.b;
    const double dv  = std::sqrt(dvx * dvx + dvy * dvy);

    double sxx_half = 0.0;
    {
      double sumt = 0.0;
      for (const auto& s : h1) sumt += s.t;
      const double mt = sumt / static_cast<double>(h1.size());
      for (const auto& s : h1) {
        const double dt = s.t - mt;
        sxx_half += dt * dt;
      }
    }
    const double sigma_v_half = sxx_half > 0.0
                                    ? sigma_pos / std::sqrt(sxx_half)
                                    : 0.0;
    const double noise_envelope = 3.0 * std::sqrt(2.0) * sigma_v_half;
    const double gate = cfg_.maneuver_dv_threshold_mps + noise_envelope;
    if (dv > gate) return est;
  }

  if (sigma_v < cfg_.min_sigma_v_m_per_s) sigma_v = cfg_.min_sigma_v_m_per_s;

  est.velocity_enu = Eigen::Vector2d(fit_x.b, fit_y.b);
  est.sigma_v_m_per_s = sigma_v;
  est.is_published = true;
  return est;
}

}  // namespace navtracker
