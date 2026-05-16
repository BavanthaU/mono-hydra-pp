#pragma once
#include <gtsam/linear/LossFunctions.h>

#include <boost/serialization/shared_ptr.hpp>
namespace gtsam {
namespace noiseModel {
namespace mEstimator {
/** The "TruncatedL2" robust error model.
 *
 *  This model has a scalar parameter "k".
 *
 * - Loss       \rho(x)          = 0.5 x²  if |x|<k, 0.5 k² + k|x-k|  otherwise
 * - Derivative \phi(x)          = x       if |x|<k, 0                otherwise
 * - Weight     w(x)             = \phi(x)/x = 1 \f$
 */
class GTSAM_EXPORT TruncatedL2 : public Base {
 protected:
  double k_;

 public:
  typedef boost::shared_ptr<TruncatedL2> shared_ptr;

  TruncatedL2(double k = 1.0, const ReweightScheme reweight = Block);
  double weight(double distance) const override;
  double loss(double distance) const override;
  void print(const std::string& s) const override;
  bool equals(const Base& expected, double tol = 1e-8) const override;
  static shared_ptr Create(double k, const ReweightScheme reweight = Block);
  double modelParameter() const { return k_; }

 private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(k_);
  }
#endif
};

/** MaxConsensus implements a binary cost, with 0 when error is less than
 *  some threshold k and 1 otherwise.
 *
 *  This model has a scalar parameter "k".
 *
 * - Loss       \f$ \rho(x) = 0 \f$ if |x|<k,    1 otherwise
 * - Derivative \f$ \phi(x) = 1 \f$ if x = k,  0 otherwise
 * - Weight     \f$ w(x) = \phi(x)/x = 1/k \f$ if x = k,  0 otherwise
 */
class GTSAM_EXPORT MaxConsensus : public Base {
 protected:
  double k_;

 public:
  typedef boost::shared_ptr<MaxConsensus> shared_ptr;

  MaxConsensus(double k = 1.0, const ReweightScheme reweight = Block);
  double weight(double distance) const override;
  double loss(double distance) const override;
  void print(const std::string& s) const override;
  bool equals(const Base& expected, double tol = 1e-8) const override;
  static shared_ptr Create(double k, const ReweightScheme reweight = Block);
  double modelParameter() const { return k_; }

 private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(k_);
  }
#endif
};

}  // namespace mEstimator
}  // namespace noiseModel
}  // namespace gtsam

namespace kimera_rpgo {
namespace robust_loss {
typedef gtsam::noiseModel::mEstimator::Null L2;
typedef gtsam::noiseModel::mEstimator::Huber Huber;
typedef gtsam::noiseModel::mEstimator::Cauchy Cauchy;
typedef gtsam::noiseModel::mEstimator::Tukey Tukey;
typedef gtsam::noiseModel::mEstimator::Welsch Welsch;
typedef gtsam::noiseModel::mEstimator::GemanMcClure GemanMcClure;
typedef gtsam::noiseModel::mEstimator::TruncatedL2 TruncatedL2;
typedef gtsam::noiseModel::mEstimator::MaxConsensus MaxConsensus;
}  // namespace robust_loss
}  // namespace kimera_rpgo
