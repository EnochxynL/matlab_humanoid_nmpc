#pragma once
#include <ocs2_core/cost/QuadraticStateCost.h>
namespace humanoid::ocp_extensions {
using namespace ocs2;
// Application metadata; all cost values/derivatives use official OCS2 methods.
class QuadraticStateCost : public ocs2::QuadraticStateCost {
public:
 explicit QuadraticStateCost(matrix_t Q):ocs2::QuadraticStateCost(Q),q_(std::move(Q)) {}
 QuadraticStateCost* clone() const override {return new QuadraticStateCost(*this);}
 void getGains(matrix_t& Q) const {Q=q_;}
protected:
 QuadraticStateCost(const QuadraticStateCost&) = default;
private:
 matrix_t q_;
};
}
