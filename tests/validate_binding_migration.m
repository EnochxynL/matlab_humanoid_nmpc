function result=validate_binding_migration()
% 完整执行链仍走原有 Simulink 接口，数值对象改为直接绑定。
root=fileparts(fileparts(mfilename('fullpath')));addpath('matlab','tools','tests');g1.initialize();
result.synchronous=validate_synchronous_backend();
result.nativeDiagnostics=validate_matlab_diagnostics();
result.replay=validate_planner_dataflow('binding_migration',true);
before=readtable(fullfile(root,'runtime','evidence','shared_walking','model_replay','planner_samples.csv'));
after=readtable(fullfile(root,'runtime','evidence','binding_migration','model_replay','planner_samples.csv'));
result.exactWindowsReplay=isequal(before{:,:},after{:,:});
assert(result.exactWindowsReplay,'Migration changed accepted Windows replay');
result.bindingUnlocked=~mislocked('ocs2_binding_mex');
assert(result.bindingUnlocked);
g1log.writeJson(fullfile(root,'runtime','evidence','binding_migration','acceptance.json'),result);
end
