//
// File: precomputationPlan.cpp
//
// MATLAB Coder version            : 25.2
// C/C++ source code generated on  : 08-Oct-2026 12:32:51
//

// Include Files
#include "precomputationPlan.h"

// Function Definitions
//
// requests: Dynamics,Cost,Constraint,SoftConstraint,Approximation (logical5).
//  Preserve original early return; Dynamics/Approximation alone do not update.
//
// Arguments    : const boolean_T requests[5]
//                boolean_T plan[2]
// Return Type  : void
//
void precomputationPlan(const boolean_T requests[5], boolean_T plan[2])
{
  if (requests[1] || requests[2] || requests[3]) {
    plan[0] = true;
    if (requests[2]) {
      plan[1] = true;
    } else {
      plan[1] = false;
    }
  } else {
    plan[0] = false;
    plan[1] = false;
  }
}

//
// File trailer for precomputationPlan.cpp
//
// [EOF]
//
