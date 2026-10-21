function result=binding_boundary_checks()
% 仅保留绑定边界和生命周期检查；不要求原型复制旧 tick/阶段协议。
cfg=g1.initialize();config=g1.plannerConfiguration(cfg.projectRoot,'',false);
h1=ocs2_binding_mex('create_problem',config);
c1=onCleanup(@()ocs2_binding_mex('destroy',h1));
h2=ocs2_binding_mex('create_problem',config);
c2=onCleanup(@()ocs2_binding_mex('destroy',h2));
result.multiple_objects=h1~=h2 && isequal(ocs2_binding_mex('dimensions',h1), ...
    ocs2_binding_mex('dimensions',h2));
ocs2_binding_mex('create_solver',h1);
result.wrong_dimension_rejected=rejected(@()ocs2_binding_mex('set_observation',h1,0,zeros(57,1),zeros(35,1),3));
result.query_before_solve_rejected=rejected(@()query(h1));
result.finite_check_rejected=rejected(@()ocs2_binding_mex('set_observation',h1,NaN,zeros(58,1),zeros(35,1),3));
clear c1;
result.stale_handle_rejected=rejected(@()dimensions(h1));
clear c2;
result.unlocked=~mislocked('ocs2_binding_mex');
end
function yes=rejected(f)
yes=false;
try,f();catch e,yes=strcmp(e.identifier,'OCS2:Binding');end
end
function query(h)
[~,~,~]=ocs2_binding_mex('evaluate_policy',h,0,zeros(58,1));
end
function dimensions(h)
ignored=ocs2_binding_mex('dimensions',h); %#ok<NASGU>
end
