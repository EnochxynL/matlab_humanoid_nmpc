#pragma once
#include <ocs2_core/cost/QuadraticStateCost.h>
namespace ocs2 {
// Application metadata; all cost values/derivatives use official OCS2 methods.
class G1QuadraticStateCost : public QuadraticStateCost {
public:
 explicit G1QuadraticStateCost(matrix_t Q):QuadraticStateCost(Q),q_(std::move(Q)) {}
 G1QuadraticStateCost* clone() const override {return new G1QuadraticStateCost(*this);}
 void getGains(matrix_t& Q) const {Q=q_;}
protected:
 G1QuadraticStateCost(const G1QuadraticStateCost&) = default;
private:
 matrix_t q_;
};
}
