#pragma once

#include <memory>
#include <vector>

#include "ports/IEstimator.hpp"
#include "ports/IMotionModel.hpp"

namespace navtracker {

// Interacting Multiple Model estimator. Carries K modes per track; each
// mode runs an EKF predict/update against its own motion model. The Track
// kinematic carrier (state, covariance) is the moment-matched projection
// of the per-mode mixture. State is unified 5-d: [px, py, vx, vy, omega].
//
// Mixing happens inside `predict`. Mode probabilities are updated inside
// `update` from the per-mode measurement likelihoods.
class ImmEstimator : public IEstimator {
 public:
  // motions.size() == K; transition_matrix is K×K with rows summing to 1
  // (pi[i][j] = P(mode j next | mode i now)); initial_mode_probabilities
  // is K, sums to 1.
  ImmEstimator(std::vector<std::shared_ptr<IMotionModel>> motions,
               Eigen::MatrixXd transition_matrix,
               Eigen::VectorXd initial_mode_probabilities,
               double init_speed_std = 10.0,
               double init_omega_std = 0.1);

  void predict(Track& track, Timestamp to) const override;
  void update(Track& track, const Measurement& z) const override;
  void softUpdate(Track& track,
                  const std::vector<Measurement>& gated_measurements,
                  const Eigen::VectorXd& betas,
                  double beta_0,
                  const PdaContext& ctx = {}) const override;
  Track initiate(const Measurement& z) const override;

 private:
  void projectMixtureToTrack(Track& track) const;

  std::vector<std::shared_ptr<IMotionModel>> motions_;
  Eigen::MatrixXd pi_;
  Eigen::VectorXd mu0_;
  double init_speed_std_;
  double init_omega_std_;
};

}  // namespace navtracker
