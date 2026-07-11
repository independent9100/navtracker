#pragma once

// IFusionRule — the pairwise state-fusion strategy port.
//
// v1 ships exactly one concrete rule: covariance intersection (CI), which is
// consistent for ANY unknown cross-correlation. The port exists so that a
// tighter, independence-exploiting rule (Bar-Shalom / Campo with modeled
// cross-covariance, or information decorrelation from full pedigree) can be
// added LATER, behind measured proof, keyed on a ProvablyIndependent pedigree
// verdict — without touching the fuser. Per the ticket: build the port, ship
// only CI. No correctness rides on a pedigree declaration being right, because
// the only shipped rule is safe for every correlation.

#include <Eigen/Core>

#include "core/t2t/CovarianceIntersection.hpp"  // CiResult, covarianceIntersect, kDefaultOmegaIterations
#include "core/t2t/Pedigree.hpp"                 // IndependenceClass

namespace navtracker::t2t {

class IFusionRule {
 public:
  virtual ~IFusionRule() = default;

  // Fuse two estimates (x1,P1),(x2,P2) of the same state into one. The
  // `independence` verdict for the pair is passed for future rules to key on;
  // the v1 CI rule ignores it (CI regardless).
  virtual CiResult fuse(const Eigen::VectorXd& x1, const Eigen::MatrixXd& P1,
                        const Eigen::VectorXd& x2, const Eigen::MatrixXd& P2,
                        IndependenceClass independence) const = 0;

  // Short stable name for output/diagnostics (e.g. "CI").
  virtual const char* name() const = 0;
};

// The sole shipped fusion rule. Deterministic: fixed golden-section iterations.
class CovarianceIntersectionRule : public IFusionRule {
 public:
  explicit CovarianceIntersectionRule(int ci_omega_iterations = kDefaultOmegaIterations)
      : iterations_(ci_omega_iterations) {}

  CiResult fuse(const Eigen::VectorXd& x1, const Eigen::MatrixXd& P1,
                const Eigen::VectorXd& x2, const Eigen::MatrixXd& P2,
                IndependenceClass independence) const override {
    (void)independence;  // v1: CI regardless of the pedigree verdict.
    return covarianceIntersect(x1, P1, x2, P2, iterations_);
  }

  const char* name() const override { return "CI"; }

 private:
  int iterations_;
};

}  // namespace navtracker::t2t
