#pragma once

#include <Eigen/Core>
#include <Eigen/LU>

#include "ports/IMeasurementNoiseModel.hpp"

namespace navtracker {

// Gaussian (no robustification). Every measurement is trusted at face
// value — the classical Kalman update. Scale is always 1.
//
// Math: identity. Assumptions: measurement noise truly Gaussian.
// Rationale: default, zero-overhead, exactly today's behaviour.
// Improve next: swap for StudentTNoiseModel where the sensor has heavy
// tails or residual clutter (EO/IR bearings, sea-clutter radar).
class GaussianNoiseModel : public IMeasurementNoiseModel {
 public:
  double covarianceScale(const Eigen::VectorXd& /*innovation*/,
                         const Eigen::MatrixXd& /*S*/) const override {
    return 1.0;
  }
};

// Student-t robust measurement model (Roth, Özkan & Gustafsson 2013).
//
// Math:
//   Model z | x ~ t_ν(ẑ, R): a Gaussian scale mixture z | x, u ~
//   N(ẑ, R/u) with u ~ Gamma(ν/2, ν/2). The posterior mean of the latent
//   scale given the innovation is
//       E[u] = (ν + d) / (ν + δ²),   δ² = yᵀ S⁻¹ y,  d = dim(z).
//   The effective measurement covariance is R / E[u], i.e. the scale on R
//   returned here is
//       s = (ν + δ²) / (ν + d).
//   For an inlier δ² ≈ d ⇒ s ≈ 1 (ordinary Kalman update). For an outlier
//   δ² ≫ d ⇒ s ≫ 1, inflating R and shrinking the gain so the outlier is
//   softly down-weighted rather than hard-rejected by a gate.
//
// Assumptions:
//   - S is the nominal innovation covariance (Gaussian R). One Gauss-step
//     approximation to the full VB iteration; sufficient for moderate
//     contamination and far cheaper.
//   - ν chosen per sensor: smaller = heavier tails / more aggressive
//     down-weighting. Roadmap suggests ν≈4 for clutter-prone ARPA/EO-IR,
//     ν→∞ recovers the Gaussian.
//
// Rationale:
//   Hard gating is all-or-nothing; a measurement just inside the gate
//   still pulls the state at full weight. Student-t down-weights smoothly
//   with the innovation, which is exactly what heavy-tailed EO/IR bearing
//   clutter needs.
//
// Improve next:
//   - Full VB iteration (recompute δ² with the inflated R, 1–3 passes).
//   - Per-sensor ν wired from SensorDefaults rather than a single value.
class StudentTNoiseModel : public IMeasurementNoiseModel {
 public:
  explicit StudentTNoiseModel(double nu) : nu_(nu) {}

  double covarianceScale(const Eigen::VectorXd& innovation,
                         const Eigen::MatrixXd& S) const override {
    const int d = static_cast<int>(innovation.size());
    if (d == 0) return 1.0;
    const double delta2 = innovation.transpose() * S.inverse() * innovation;
    const double s = (nu_ + delta2) / (nu_ + static_cast<double>(d));
    return s < 1.0 ? 1.0 : s;  // never down-trust below Gaussian
  }

  double nu() const { return nu_; }

 private:
  double nu_;
};

}  // namespace navtracker
