function plan = precomputationPlan(requests)
%#codegen
% requests: Dynamics,Cost,Constraint,SoftConstraint,Approximation (logical5).
% Preserve original early return; Dynamics/Approximation alone do not update.
plan = false(2,1);
plan(1) = requests(2) || requests(3) || requests(4);
plan(2) = plan(1) && requests(3);
end
