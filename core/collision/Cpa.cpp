#include "core/collision/Cpa.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navtracker {

namespace {

double standardNormalCdf(double x) {
  return 0.5 * std::erfc(-x / std::sqrt(2.0));
}

constexpr double kEpsDv2 = 1e-12;    // |dv|^2 below this -> parallel
constexpr double kEpsCpa = 1.0;      // cpa below this -> head-on fallback

}  // namespace


CpaResult computeCpa(const Track& a, const Track& b, Timestamp t_ref) {
  // Extrapolate each track from its own last_update to t_ref using
  // state(2..3) as the CV velocity.
  const double dt_a = t_ref.secondsSince(a.last_update);
  const double dt_b = t_ref.secondsSince(b.last_update);
  const Eigen::Vector2d pa(a.state(0) + a.state(2) * dt_a,
                           a.state(1) + a.state(3) * dt_a);
  const Eigen::Vector2d pb(b.state(0) + b.state(2) * dt_b,
                           b.state(1) + b.state(3) * dt_b);
  const Eigen::Vector2d va(a.state(2), a.state(3));
  const Eigen::Vector2d vb(b.state(2), b.state(3));
  const Eigen::Vector2d dp = pa - pb;
  const Eigen::Vector2d dv = va - vb;

  CpaResult r;
  const double dv2 = dv.dot(dv);
  if (dv2 < kEpsDv2) {
    // Parallel velocities -> constant separation. Not diverging.
    r.tcpa_seconds = 0.0;
    r.cpa_distance_m = dp.norm();
    r.is_diverging = false;
    return r;
  }
  const double t_cpa_raw = -dp.dot(dv) / dv2;
  if (t_cpa_raw <= 0.0) {
    // Closest approach is in the past (or exactly now). Report the
    // CURRENT distance, not the past minimum.
    r.tcpa_seconds = 0.0;
    r.cpa_distance_m = dp.norm();
    r.is_diverging = (t_cpa_raw < 0.0);
    return r;
  }
  r.tcpa_seconds = t_cpa_raw;
  const Eigen::Vector2d dp_at_cpa = dp + dv * t_cpa_raw;
  r.cpa_distance_m = dp_at_cpa.norm();
  r.is_diverging = false;
  return r;
}

CpaPrediction computeCpaWithUncertainty(const Track& a, const Track& b,
                                        Timestamp t_ref,
                                        double d_threshold_m) {
  // 1. Extrapolate mean states to t_ref (mirrors computeCpa).
  const double dt_a = t_ref.secondsSince(a.last_update);
  const double dt_b = t_ref.secondsSince(b.last_update);
  const Eigen::Vector2d pa(a.state(0) + a.state(2) * dt_a,
                           a.state(1) + a.state(3) * dt_a);
  const Eigen::Vector2d pb(b.state(0) + b.state(2) * dt_b,
                           b.state(1) + b.state(3) * dt_b);
  const Eigen::Vector2d va(a.state(2), a.state(3));
  const Eigen::Vector2d vb(b.state(2), b.state(3));
  const Eigen::Vector2d dp = pa - pb;
  const Eigen::Vector2d dv = va - vb;

  // 2. Build joint covariance Σ (8x8 block-diag).
  Eigen::Matrix<double, 8, 8> Sigma = Eigen::Matrix<double, 8, 8>::Zero();
  Sigma.block<4, 4>(0, 0) = a.covariance.topLeftCorner<4, 4>();
  Sigma.block<4, 4>(4, 4) = b.covariance.topLeftCorner<4, 4>();

  // 3. Build ∂dp/∂x and ∂dv/∂x as 2x8 matrices (per spec §4.3).
  Eigen::Matrix<double, 2, 8> dDp_dx = Eigen::Matrix<double, 2, 8>::Zero();
  dDp_dx.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity();
  dDp_dx.block<2, 2>(0, 2) = dt_a * Eigen::Matrix2d::Identity();
  dDp_dx.block<2, 2>(0, 4) = -Eigen::Matrix2d::Identity();
  dDp_dx.block<2, 2>(0, 6) = -dt_b * Eigen::Matrix2d::Identity();

  Eigen::Matrix<double, 2, 8> dDv_dx = Eigen::Matrix<double, 2, 8>::Zero();
  dDv_dx.block<2, 2>(0, 2) =  Eigen::Matrix2d::Identity();
  dDv_dx.block<2, 2>(0, 6) = -Eigen::Matrix2d::Identity();

  const double dv2 = dv.dot(dv);
  const double t_cpa_raw = (dv2 > 0.0) ? -dp.dot(dv) / dv2
                                       : 0.0;

  CpaPrediction r;
  r.d_threshold_m = d_threshold_m;

  // 4. Branch A — parallel velocities or past CPA.
  if (dv2 < kEpsDv2 || t_cpa_raw <= 0.0) {
    const double cur_dist = dp.norm();
    r.tcpa_seconds = 0.0;
    r.cpa_distance_m = cur_dist;
    r.sigma_tcpa_seconds = std::numeric_limits<double>::infinity();
    r.is_diverging = (t_cpa_raw < 0.0);

    // σ_cpa from current dp covariance via direction-projection.
    const Eigen::Matrix<double, 2, 2> cov_dp =
        dDp_dx * Sigma * dDp_dx.transpose();
    double sigma_cpa = 0.0;
    if (cur_dist > 0.0) {
      const Eigen::Vector2d dp_hat = dp / cur_dist;
      const double var_cpa =
          (dp_hat.transpose() * cov_dp * dp_hat)(0, 0);
      sigma_cpa = std::sqrt(std::max(var_cpa, 0.0));
    } else {
      sigma_cpa = std::sqrt(std::max(0.5 * cov_dp.trace(), 0.0));
    }
    r.sigma_cpa_m = sigma_cpa;
    r.probability_below_threshold = (sigma_cpa > 0.0)
        ? standardNormalCdf((d_threshold_m - cur_dist) / sigma_cpa)
        : (cur_dist < d_threshold_m ? 1.0 :
           (cur_dist > d_threshold_m ? 0.0 : 0.5));
    return r;
  }

  // 5. General case — compute Jacobians per spec §4.3.
  const double t_cpa = t_cpa_raw;
  const Eigen::Vector2d p_cpa = dp + dv * t_cpa;
  const double cpa = p_cpa.norm();

  // ∂t_cpa/∂x = (1x8)
  // numerator: -[dvᵀ · ∂dp/∂x + dpᵀ · ∂dv/∂x]
  Eigen::Matrix<double, 1, 8> num = -(dv.transpose() * dDp_dx
                                       + dp.transpose() * dDv_dx);
  Eigen::Matrix<double, 1, 8> chain = 2.0 * t_cpa
                                        * (dv.transpose() * dDv_dx);
  Eigen::Matrix<double, 1, 8> J_tcpa = (num + chain) / dv2;

  // ∂p_cpa/∂x = ∂dp/∂x + t_cpa · ∂dv/∂x + dv · J_tcpa   (2x8)
  Eigen::Matrix<double, 2, 8> J_p_cpa = dDp_dx + t_cpa * dDv_dx
                                       + dv * J_tcpa;

  // σ²_tcpa = J_tcpa · Σ · J_tcpaᵀ
  const double var_tcpa = (J_tcpa * Sigma * J_tcpa.transpose())(0, 0);

  // σ²_cpa via direction projection (or isotropic fallback if cpa ≈ 0).
  const Eigen::Matrix<double, 2, 2> cov_p_cpa =
      J_p_cpa * Sigma * J_p_cpa.transpose();
  double sigma_cpa;
  if (cpa > kEpsCpa) {
    const Eigen::Vector2d p_cpa_hat = p_cpa / cpa;
    const double var_cpa =
        (p_cpa_hat.transpose() * cov_p_cpa * p_cpa_hat)(0, 0);
    sigma_cpa = std::sqrt(std::max(var_cpa, 0.0));
  } else {
    sigma_cpa = std::sqrt(std::max(0.5 * cov_p_cpa.trace(), 0.0));
  }

  r.cpa_distance_m = cpa;
  r.sigma_cpa_m = sigma_cpa;
  r.tcpa_seconds = t_cpa;
  r.sigma_tcpa_seconds = std::sqrt(std::max(var_tcpa, 0.0));
  r.is_diverging = false;
  r.probability_below_threshold = (sigma_cpa > 0.0)
      ? standardNormalCdf((d_threshold_m - cpa) / sigma_cpa)
      : (cpa < d_threshold_m ? 1.0 :
         (cpa > d_threshold_m ? 0.0 : 0.5));
  return r;
}

}  // namespace navtracker
