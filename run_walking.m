function out=run_walking(formulation)
% Genuine 4.8 s plant run; no ROS2 endpoints or serialized reference IPC.
root=fileparts(mfilename('fullpath'));cd(root);addpath(fullfile(root,'matlab'));
cfg=g1.initialize();
if nargin<1,formulation='whole_body';end
if strcmp(formulation,'centroidal'),out=run_centroidal_walking();return;end
assert(strcmp(formulation,'whole_body'),'Unknown MPC formulation');
cacheCleanup=g1.useBuildCache(); %#ok<NASGU>
assert(exist('ocs2_binding_mex','file')==3,'Build native/scripts/build_mex.sh first.');
assert(isfile(cfg.modelFile),'Missing models/g1_walking.slx.');
folder=fullfile(root,'runtime',['walking_' char(datetime('now','TimeZone','Asia/Shanghai','Format','yyyyMMdd_HHmmss'))]);mkdir(folder);
env=jsondecode(fileread(fullfile(root,'config/controller_environment.json')));
keys=fieldnames(env);old=cell(size(keys));for k=1:numel(keys),old{k}=getenv(keys{k});setenv(keys{k},env.(keys{k}));end
cleanup=onCleanup(@()finish(keys,old));diary(fullfile(folder,'matlab.log'));
g1log.writeJson(fullfile(root,'runtime/current_run.json'),struct('folder',folder,'status','STARTING'));
if ispc
 [~,memorySystem]=memory;mem=jsonencode(memorySystem);
else
 [~,mem]=system('cat /proc/meminfo');
end
fid=fopen(fullfile(folder,'memory_before.txt'),'w');fprintf(fid,'%s',mem);fclose(fid);

session=struct('root',root,'folder',folder,'duration',4.8,'transport','IN_PROCESS_MEX','ocp','accepted OCS2 SQP','query_lead_s',.005,'physical_period_s',.002,'environment',env,'baseline_sha','7565b42006be8b0f8c7b21f441f50c7b773431d9');
g1log.writeJson(fullfile(folder,'configuration.json'),session);
wall=tic;g1.plannerMex('init',g1.plannerConfiguration(root,folder));startup=toc(wall);session.startup_wall_s=startup;
if ~ispc,open_system(fullfile(root,'models',[cfg.modelName '.slx']));end
in=g1.walkingSimulation(cfg,'whole_body',4.8,false);
if exist('layout.LayoutStateManager','class')
 layoutHold=layout.LayoutStateManager.instance().getScopedLayoutListenerDisabler(cfg.modelName); %#ok<NASGU>
end
g1log.writeJson(fullfile(root,'runtime/current_run.json'),struct('folder',folder,'status','SIMULATING'));
fprintf('Starting genuine Simscape walking: StopTime=4.8 s; evidence=%s\n',folder);
wall=tic;out=sim(in);session.execution_wall_s=toc(wall);
g1.plannerMex('flush');
clear layoutHold;
save(fullfile(folder,'simulation_output.mat'),'out','session','-v7.3');
g1log.writeJson(fullfile(folder,'configuration.json'),session);
if ~isempty(out.ErrorMessage)
 g1log.writeJson(fullfile(root,'runtime/current_run.json'),struct('folder',folder,'status','FAILED','error',out.ErrorMessage));
 error('G1:InprocessSimulation','%s',out.ErrorMessage);
end
logs=g1log.collectSignals(out);g1log.exportPhysics(logs,folder);
state=logs.get('PlantState').Values;clock=state.sim_time.Time;
assert(abs(clock(end)-4.8)<1e-12,'Simulation ended before 4.8 s.');
fields=fieldnames(state);for k=1:numel(fields),assert(all(isfinite(state.(fields{k}).Data),'all'),'Nonfinite physical state.');end
% Independent action assembly check against the actual planner/ID output.
planSignal=out.yout{6}.Values;plan=g1log.signalMatrix(planSignal);actions=logs.get('RobotJointAction').Values;
[planTimes,uniqueRows]=unique(planSignal.Time,'last');
plan=interp1(planTimes,plan(uniqueRows,:),actions.q_des.Time,'previous');
assert(all(isfinite(plan),'all'),'Action sample falls outside planner output coverage.');
M=zeros(29,23);for k=1:23,M(cfg.mpcIndices(k),k)=1;end
expected={plan(:,7:29)*M',plan(:,36:58)*M',[],[],plan(:,94:116)*M'};
kp=100*ones(1,29);kp(cfg.mpcIndices)=1200;kd=ones(1,29);kd(cfg.mpcIndices)=10;
expected{3}=repmat(kp,size(plan,1),1);expected{4}=repmat(kd,size(plan,1),1);
maximum=0;for k=1:5,a=g1log.signalMatrix(actions.(cfg.actionFields{k}));assert(isequal(size(a),size(expected{k})));maximum=max(maximum,max(abs(a-expected{k}),[],'all'));end
assert(maximum<1e-12,'Dataflow joint action differs from planner/ID contract.');
identity=readtable(fullfile(folder,'planner_samples.csv'));
assert(isequal(identity.control_tick,(0:2400)'),'Missing or repeated physical control tick.');
assert(all(identity.canonical_time==double(identity.control_tick)*.002),'Canonical time identity changed.');
rawError=max(abs(identity.raw_time-identity.canonical_time));assert(rawError<=1e-12);
assert(max(abs(identity.value117-identity.canonical_time-.005))<=1e-12,'Policy query lead changed.');
solves=readtable(fullfile(folder,'solver_timing.csv'));
p=g1log.signalMatrix(state.base_position_world);q=g1log.signalMatrix(state.base_orientation_wxyz);
pitch=asin(max(-1,min(1,2*(q(:,1).*q(:,3)-q(:,4).*q(:,2)))));
result=struct('passed',true,'duration_s',clock(end),'samples',numel(clock),'action_mapping_max_error',maximum, ...
 'position_start',p(1,:),'position_end',p(end,:),'displacement_m',p(end,:)-p(1,:), ...
 'height_range_m',[min(p(:,3)) max(p(:,3))],'max_abs_pitch_deg',max(abs(rad2deg(pitch))), ...
 'mpc_updates',height(solves),'control_ticks',height(identity),'raw_minus_canonical_max_s',rawError, ...
 'startup_wall_s',startup,'execution_wall_s',session.execution_wall_s,'human_visual','PENDING');
g1log.writeJson(fullfile(folder,'verification.json'),result);disp(result);
g1log.writeJson(fullfile(root,'runtime/current_run.json'),struct('folder',folder,'status','NUMERICAL_PASS_VISUAL_PENDING'));
f=figure('Name','G1 in-process walking verification','Visible','off');tiledlayout(3,1);
nexttile;plot(clock,p);legend('x','y','z');ylabel('Position (m)');grid on;
nexttile;plot(clock,rad2deg(pitch));ylabel('Pitch (deg)');grid on;
contact=logs.get('ContactDiagnostics').Values;
nexttile;plot(contact.Left.total_force_world.Time,g1log.signalMatrix(contact.Left.total_force_world)*[0;0;1]);hold on;
plot(contact.Right.total_force_world.Time,g1log.signalMatrix(contact.Right.total_force_world)*[0;0;1]);
legend('Left','Right');ylabel('Actual foot Fz (N)');xlabel('Physical time (s)');grid on;
exportgraphics(f,fullfile(folder,'physical_summary.png'));close(f);
fprintf('Run complete; Mechanics Explorer replay and evidence: %s\n',folder);
 function finish(envKeys,envPrevious)
  g1.plannerMex('close');diary off;for j=1:numel(envKeys),setenv(envKeys{j},envPrevious{j});end
 end
end
