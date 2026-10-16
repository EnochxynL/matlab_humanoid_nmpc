#pragma once
#include <ocs2_core/constraint/StateInputConstraint.h>
namespace humanoid::ocp_extensions {
class ActiveConstraint {
public:
 virtual ~ActiveConstraint() = default;
 virtual void setActive(bool) = 0;
 virtual bool getActive() const = 0;
};
inline bool active(const ocs2::StateInputConstraint& c) {
 auto* a = dynamic_cast<const ActiveConstraint*>(&c);
 return a ? a->getActive() : true;
}
inline void setActive(ocs2::StateInputConstraint& c, bool enabled) {
 if(auto* a = dynamic_cast<ActiveConstraint*>(&c)) a->setActive(enabled);
 // Preserve original default base implementation, which was a no-op.
}
}
