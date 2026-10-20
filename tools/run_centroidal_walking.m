function out = run_centroidal_walking(duration, standing)
%RUN_CENTROIDAL_WALKING Centroidal MPC + 独立逆动力学的真实多体闭环。
% run_centroidal_walking(1,true) 先验证站立；默认运行 4.8 s 行走。
if nargin<1, duration=4.8; end
if nargin<2, standing=false; end
assert(isscalar(duration) && isfinite(duration) && duration>0 && ...
    abs(duration/.002-round(duration/.002))<1e-10);
assert(islogical(standing) && isscalar(standing));
cfg=g1.initialize();root=cfg.projectRoot;
cacheCleanup=g1.useBuildCache(); %#ok<NASGU>
model='g1_centroidal_walking';
parameters=jsondecode(fileread(fullfile(root,'config','centroidal','execution.json')));
folder=fullfile(root,'runtime',['centroidal_' char(datetime('now', ...
    'TimeZone','Asia/Taipei','Format','yyyyMMdd_HHmmss_SSS'))]);
mkdir(folder);
session=struct('formulation','centroidal','duration_s',duration,'standing',standing, ...
    'execution',parameters,'folder',folder,'physical_simulation',true);
g1log.writeJson(fullfile(folder,'configuration.json'),session);
g1log.writeJson(fullfile(root,'runtime','current_run.json'),struct('folder',folder,'status','STARTING'));
config=g1.plannerConfiguration(root,folder,true,'centroidal');config.executionEnabled=true;
g1.plannerMex('init',config);
cleanup=onCleanup(@()g1.plannerMex('close'));
open_system(fullfile(root,'models',[model '.slx']));
in=Simulink.SimulationInput(model);
in=in.setModelParameter('StopTime',num2str(duration,17),'MaxStep','0.0005', ...
    'OutputOption','SpecifiedOutputTimes','OutputTimes',mat2str(0:cfg.driveTs:duration,17), ...
    'SimMechanicsOpenEditorOnUpdate','on','CaptureErrors','on');
names={'g1_q0','g1_qd0','g1_base_p0','g1_base_rpy0','g1_gravity','g1_standing', ...
    'centroidal_height','centroidal_forward_command'};
values={cfg.initialQ,cfg.initialQd,cfg.initialPosition,cfg.initialRpy,cfg.gravity, ...
    double(standing),parameters.command_height_m,parameters.forward_command};
for k=1:numel(names),in=in.setVariable(names{k},values{k},'Workspace',model);end
kp=parameters.fixed_joint_kp*ones(29,1);kp(cfg.mpcIndices)=parameters.joint_position_kp;
kd=parameters.fixed_joint_kd*ones(29,1);kd(cfg.mpcIndices)=parameters.joint_velocity_kd;
in=in.setVariable('g1_kp',kp,'Workspace',model);
in=in.setVariable('g1_kd',kd,'Workspace',model);
in=in.setVariable('accelerationKp',parameters.acceleration_feedback_kp*ones(23,1), ...
    'Workspace','g1_centroidal_execution');
in=in.setVariable('accelerationKd',parameters.acceleration_feedback_kd*ones(23,1), ...
    'Workspace','g1_centroidal_execution');
if exist('layout.LayoutStateManager','class')
    layoutHold=layout.LayoutStateManager.instance().getScopedLayoutListenerDisabler(model); %#ok<NASGU>
end
fprintf('Centroidal physical run: %.3f s, standing=%d, evidence=%s\n',duration,standing,folder);
g1log.writeJson(fullfile(root,'runtime','current_run.json'),struct('folder',folder,'status','SIMULATING'));
wall=tic;out=sim(in);session.execution_wall_s=toc(wall);
g1.plannerMex('flush');clear cleanup;
save(fullfile(folder,'simulation_output.mat'),'out','session','-v7.3');
logs=g1log.collectSignals(out);g1log.exportPhysics(logs,folder);
if ~isempty(out.ErrorMessage)
    g1log.writeJson(fullfile(folder,'verification.json'), ...
        struct('passed',false,'error',out.ErrorMessage,'execution_wall_s',session.execution_wall_s));
    g1log.writeJson(fullfile(root,'runtime','current_run.json'), ...
        struct('folder',folder,'status','FAILED','error',out.ErrorMessage));
    error('G1:CentroidalSimulation','%s',out.ErrorMessage);
end
state=logs.get('PlantState').Values;time=state.sim_time.Time;
p=g1log.signalMatrix(state.base_position_world);
q=g1log.signalMatrix(state.base_orientation_wxyz);
pitch=asin(max(-1,min(1,2*(q(:,1).*q(:,3)-q(:,4).*q(:,2)))));
roll=atan2(2*(q(:,1).*q(:,2)+q(:,3).*q(:,4)),1-2*(q(:,2).^2+q(:,3).^2));
samples=readtable(fullfile(folder,'centroidal_samples.csv'));
solves=readtable(fullfile(folder,'solver_timing.csv'));
planSignal=out.yout{6}.Values;plan=g1log.signalMatrix(planSignal);
actions=logs.get('RobotJointAction').Values;
[planTimes,uniqueRows]=unique(planSignal.Time,'last');
plan=interp1(planTimes,plan(uniqueRows,:),actions.q_des.Time,'previous');
M=zeros(29,23);for k=1:23,M(cfg.mpcIndices(k),k)=1;end
expected={plan(:,13:35)*M',plan(:,48:70)*M',repmat(kp',size(plan,1),1), ...
    repmat(kd',size(plan,1),1),plan(:,71:93)*M'};
mappingError=0;
for k=1:5
    actual=g1log.signalMatrix(actions.(cfg.actionFields{k}));
    assert(isequal(size(actual),size(expected{k})));
    mappingError=max(mappingError,max(abs(actual-expected{k}),[],'all'));
end
finite=all(isfinite(plan),'all');
fields=fieldnames(state);
for k=1:numel(fields),finite=finite && all(isfinite(state.(fields{k}).Data),'all');end
tickIdentity=isequal(samples.control_tick,(0:round(duration/.002))') && ...
    all(samples.canonical_time==samples.control_tick*.002);
requestedTicks=unique(ceil(((0:floor(duration*80))/80-1e-12)/.002));
requestedTicks(requestedTicks<0)=0;
scheduleIdentity=isequal(solves.solver_target_control_time_s,requestedTicks'*.002);
upright=min(p(:,3))>.65 && max(abs(pitch))<deg2rad(25) && max(abs(roll))<deg2rad(25);
displacement=p(end,:)-p(1,:);
progress=standing || duration<3 || displacement(1)>.1;
singleSupportTicks=sum(ismember(samples.value93,[1,2]));
contact=logs.get('ContactDiagnostics').Values;
left=g1log.signalMatrix(contact.Left.in_contact);
right=g1log.signalMatrix(contact.Right.in_contact);
leftOnly=sum(left~=0 & right==0);rightOnly=sum(right~=0 & left==0);
stepping=standing || duration<3 || (singleSupportTicks>0 && leftOnly>0 && rightOnly>0);
result=struct('passed',finite && tickIdentity && scheduleIdentity && upright && progress && stepping && mappingError<1e-12, ...
    'duration_s',time(end),'standing',standing,'height_range_m',[min(p(:,3)),max(p(:,3))], ...
    'displacement_m',displacement,'max_abs_pitch_deg',max(abs(rad2deg(pitch))), ...
    'max_abs_roll_deg',max(abs(rad2deg(roll))),'action_mapping_max_error',mappingError, ...
    'control_ticks',height(samples),'mpc_updates',height(solves),'physical_simulation',true, ...
    'planned_single_support_ticks',singleSupportTicks, ...
    'measured_left_only_samples',leftOnly,'measured_right_only_samples',rightOnly, ...
    'execution_wall_s',session.execution_wall_s,'human_visual','PENDING');
g1log.writeJson(fullfile(folder,'verification.json'),result);disp(result);
g1log.writeJson(fullfile(root,'runtime','current_run.json'), ...
    struct('folder',folder,'status','COMPLETED','passed',result.passed));
assert(abs(time(end)-duration)<1e-12 && result.passed,'Centroidal physical validation failed.');
end
