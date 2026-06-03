#include "core/own_ship/UereEstimator.hpp"

#include <cmath>

namespace navtracker {
namespace {

struct LinearFit {
  double a{0.0};      // intercept
  double b{0.0};      // slope (velocity component)
  double ss_res{0.0}; // sum residual^2
};

// Fit p(t) = a + b * (t - t0). Returns fit + SSR. Assumes samples.size() >= 2.
template <class Getter>
LinearFit fitAxis(const std::deque<UereEstimator::Sample>& samples,
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

UereEstimator::UereEstimator(UereEstimatorConfig cfg) : cfg_(cfg) {}

void UereEstimator::observe(Timestamp t, double x_enu, double y_enu) {
  samples_.push_back({static_cast<double>(t.nanos()) * 1e-9, x_enu, y_enu});
  while (samples_.size() > cfg_.window_size) samples_.pop_front();
}

UereEstimate UereEstimator::current() const {
  UereEstimate est;
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

  // Maneuver gating: split window in halves, fit velocity in each, reject
  // if |delta-v| > threshold. The threshold is augmented by the expected
  // velocity-difference noise induced by GPS jitter (3-sigma envelope) so
  // that the gate fires on genuine maneuvers rather than on noisy
  // velocity estimates from a noisy window.
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

    // Expected per-axis velocity uncertainty in one half: sigma_pos /
    // sqrt(sxx_half). dv combines two halves and two axes — expected
    // ||delta-v|| ~ sqrt(2) * sigma_v_per_half * sqrt(2) for 2D Rayleigh
    // mean, but use 3-sigma envelope: 3 * sqrt(2) * sigma_v_per_half.
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

  double sigma = sigma_pos;
  if (sigma < cfg_.min_sigma_m) sigma = cfg_.min_sigma_m;
  est.sigma_m = sigma;
  est.is_published = true;
  return est;
}

}  // namespace navtracker
