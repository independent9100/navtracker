#include "adapters/foxglove/Geometry.hpp"
#include <cmath>
#include <functional>
#include <Eigen/Eigenvalues>

namespace navtracker::foxglove {

std::vector<Pt> covarianceEllipse(const Eigen::Vector2d& c, const Eigen::Matrix2d& cov,
                                  double k, int n) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cov);
  Eigen::Vector2d lam = es.eigenvalues().cwiseMax(0.0);   // clamp tiny negatives
  Eigen::Matrix2d V = es.eigenvectors();
  const double a = k * std::sqrt(lam(0)), b = k * std::sqrt(lam(1));
  std::vector<Pt> out;
  out.reserve(n + 1);
  for (int i = 0; i <= n; ++i) {                          // <= n closes the loop
    const double t = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(n);
    Eigen::Vector2d e(a * std::cos(t), b * std::sin(t));  // in eigenbasis
    Eigen::Vector2d p = c + V * e;                        // back to ENU
    out.push_back({p.x(), p.y(), 0.0});
  }
  return out;
}

std::vector<Pt> bearingWedge(const Eigen::Vector2d& s, double alpha, double sigma,
                             double length, double k) {
  const double a1 = alpha + k * sigma, a2 = alpha - k * sigma;
  Eigen::Vector2d e1 = s + length * Eigen::Vector2d(std::cos(a1), std::sin(a1));
  Eigen::Vector2d e2 = s + length * Eigen::Vector2d(std::cos(a2), std::sin(a2));
  return {{e1.x(), e1.y(), 0.0}, {s.x(), s.y(), 0.0}, {e2.x(), e2.y(), 0.0}};
}

Rgba colorForSensor(SensorKind sensor, const std::string& source_id) {
  // Hash (kind, source_id) into a hue; fixed S/V. Deterministic across runs.
  const std::size_t h = std::hash<int>{}(static_cast<int>(sensor)) * 1000003u
                      ^ std::hash<std::string>{}(source_id);
  const double hue = static_cast<double>(h % 360u);       // degrees
  const double s = 0.65, v = 0.95, c = v * s, x = c * (1 - std::abs(std::fmod(hue / 60.0, 2.0) - 1));
  const double m = v - c;
  double r = 0, g = 0, bl = 0;
  if (hue < 60)      { r = c; g = x; }
  else if (hue < 120){ r = x; g = c; }
  else if (hue < 180){ g = c; bl = x; }
  else if (hue < 240){ g = x; bl = c; }
  else if (hue < 300){ r = x; bl = c; }
  else               { r = c; bl = x; }
  return {r + m, g + m, bl + m, 1.0};
}

}  // namespace navtracker::foxglove
