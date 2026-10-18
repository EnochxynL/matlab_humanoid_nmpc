#pragma once
#include <ocs2_core/cost/QuadraticStateInputCost.h>
namespace humanoid::ocp_extensions {
using namespace ocs2;
// Application metadata; all cost values/derivatives use official OCS2 methods.
class QuadraticStateInputCost : public ocs2::QuadraticStateInputCost {
public:
 QuadraticStateInputCost(matrix_t Q,matrix_t R,matrix_t P=matrix_t()):ocs2::QuadraticStateInputCost(Q,R,P),q_(std::move(Q)),r_(std::move(R)),p_(std::move(P)) {}
 QuadraticStateInputCost* clone() const override {return new QuadraticStateInputCost(*this);}
 void getGains(matrix_t& Q,matrix_t& R,matrix_t& P) const {Q=q_;R=r_;P=p_;}
protected:
 QuadraticStateInputCost(const QuadraticStateInputCost&) = default;
private:
 matrix_t q_,r_,p_;
};
}
