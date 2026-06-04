#include "core/tracking/DatumShift.hpp"

namespace navtracker {

namespace {
void shiftPosition(double& px, double& py,
                   const geo::Datum& old_d, const geo::Datum& new_d) {
  const auto geo = old_d.toGeodetic(Eigen::Vector3d(px, py, 0.0));
  const auto enu_new = new_d.toEnu(geo);
  px = enu_new.x();
  py = enu_new.y();
}

void rotateVelocityInPlace(double& vx, double& vy,
                           const Eigen::Matrix2d& R) {
  const Eigen::Vector2d v(vx, vy);
  const Eigen::Vector2d v_rot = R * v;
  vx = v_rot.x();
  vy = v_rot.y();
}

void rotateCovarianceInPlace(Eigen::MatrixXd& cov,
                             const Eigen::Matrix2d& R) {
  if (cov.rows() < 4 || cov.cols() < 4) return;  // smaller states skip
  Eigen::Matrix4d Rbar = Eigen::Matrix4d::Zero();
  Rbar.topLeftCorner<2, 2>()     = R;
  Rbar.bottomRightCorner<2, 2>() = R;
  cov.topLeftCorner<4, 4>() =
      Rbar * cov.topLeftCorner<4, 4>().eval() * Rbar.transpose();
}
}  // namespace

void shiftTracksOnDatumChange(TrackManager& mgr,
                              const geo::Datum& old_datum,
                              const geo::Datum& new_datum) {
  const Eigen::Matrix2d R = geo::datumAxisRotation(old_datum, new_datum);
  for (auto& t : mgr.mutableTracks()) {
    if (t.state.size() >= 2) {
      shiftPosition(t.state(0), t.state(1), old_datum, new_datum);
    }
    if (t.state.size() >= 4) {
      rotateVelocityInPlace(t.state(2), t.state(3), R);
    }
    rotateCovarianceInPlace(t.covariance, R);

    // IMM modes: shift each column.
    if (t.imm_means.size() > 0) {
      for (int k = 0; k < t.imm_means.cols(); ++k) {
        if (t.imm_means.rows() >= 2) {
          double px = t.imm_means(0, k), py = t.imm_means(1, k);
          shiftPosition(px, py, old_datum, new_datum);
          t.imm_means(0, k) = px;
          t.imm_means(1, k) = py;
        }
        if (t.imm_means.rows() >= 4) {
          double vx = t.imm_means(2, k), vy = t.imm_means(3, k);
          rotateVelocityInPlace(vx, vy, R);
          t.imm_means(2, k) = vx;
          t.imm_means(3, k) = vy;
        }
      }
    }
    for (auto& cov_k : t.imm_covariances) {
      Eigen::MatrixXd cov_dyn = cov_k;
      rotateCovarianceInPlace(cov_dyn, R);
      cov_k = cov_dyn;
    }

    // Particles: shift each column's position.
    if (t.particles.size() > 0 && t.particles.rows() >= 2) {
      for (int k = 0; k < t.particles.cols(); ++k) {
        double px = t.particles(0, k), py = t.particles(1, k);
        shiftPosition(px, py, old_datum, new_datum);
        t.particles(0, k) = px;
        t.particles(1, k) = py;
        if (t.particles.rows() >= 4) {
          double vx = t.particles(2, k), vy = t.particles(3, k);
          rotateVelocityInPlace(vx, vy, R);
          t.particles(2, k) = vx;
          t.particles(3, k) = vy;
        }
      }
    }
  }
}

}  // namespace navtracker
