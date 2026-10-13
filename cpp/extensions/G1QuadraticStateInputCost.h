#pragma once
#include <ocs2_core/cost/QuadraticStateInputCost.h>
namespace ocs2 {
// Application metadata; all cost values/derivatives use official OCS2 methods.
class G1QuadraticStateInputCost : public QuadraticStateInputCost {
public:
 G1QuadraticStateInputCost(matrix_t Q,matrix_t R,matrix_t P=matrix_t()):QuadraticStateInputCost(Q,R,P),q_(std::move(Q)),r_(std::move(R)),p_(std::move(P)) {}
 G1QuadraticStateInputCost* clone() const override {return new G1QuadraticStateInputCost(*this);}
 void getGains(matrix_t& Q,matrix_t& R,matrix_t& P) const {Q=q_;R=r_;P=p_;}
protected:
 G1QuadraticStateInputCost(const G1QuadraticStateInputCost&) = default;
private:
 matrix_t q_,r_,p_;
};
}
