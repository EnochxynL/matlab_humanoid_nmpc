function result=validate_binding_walking()
% 短多体闭环检查迁移后的真实模型接线，完整控制数据由冻结回放覆盖。
cfg=g1.initialize();root=cfg.projectRoot;cacheCleanup=g1.useBuildCache(); %#ok<NASGU>
folder=fullfile(root,'runtime','evidence','binding_migration','whole_body_physics');
g1.plannerMex('init',g1.plannerConfiguration(root,folder));cleanup=onCleanup(@()g1.plannerMex('close'));
in=g1.walkingSimulation(cfg,'whole_body',.1,true);
in=in.setModelParameter('SimMechanicsOpenEditorOnUpdate','off');
out=sim(in);assert(isempty(out.ErrorMessage),'%s',out.ErrorMessage);
g1.plannerMex('flush');clear cleanup;
logs=g1log.collectSignals(out);g1log.exportPhysics(logs,folder);
state=logs.get('PlantState').Values;p=g1log.signalMatrix(state.base_position_world);
assert(all(isfinite(p),'all') && min(p(:,3))>.65);
accepted=load(fullfile(root,'runtime','evidence','shared_walking','whole_body','simulation_output.mat'),'out');
assert(isequal(g1log.signalMatrix(out.yout{6}.Values),g1log.signalMatrix(accepted.out.yout{6}.Values)));
previous=g1log.collectSignals(accepted.out);old=previous.get('PlantState').Values;
assert(isequal(p,g1log.signalMatrix(old.base_position_world)));
result.whole_body=struct('passed',true,'duration_s',.1,'policy_torque_exact',true,'position_exact',true);
save(fullfile(folder,'simulation_output.mat'),'out','-v7.3');
run_centroidal_walking(.1,true);
current=jsondecode(fileread(fullfile(root,'runtime','current_run.json')));
result.centroidal=jsondecode(fileread(fullfile(current.folder,'verification.json')));
accepted=load(fullfile(root,'runtime','evidence','shared_walking','centroidal','simulation_output.mat'),'out');
actual=load(fullfile(current.folder,'simulation_output.mat'),'out');
assert(isequal(g1log.signalMatrix(actual.out.yout{6}.Values),g1log.signalMatrix(accepted.out.yout{6}.Values)));
result.centroidal.policy_torque_exact=true;
g1log.writeJson(fullfile(root,'runtime','evidence','binding_migration','physical_acceptance.json'),result);
end
