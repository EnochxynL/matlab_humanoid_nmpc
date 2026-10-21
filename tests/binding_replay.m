function result=binding_replay(formulation)
%BINDING_REPLAY 用现有冻结输入比较 Controller 与直接原生绑定。
% 不做物理仿真。所有调度、reset 目标及查询提前量明确写在 MATLAB。
root=fileparts(fileparts(mfilename('fullpath')));cfg=g1.initialize();
golden=load(fullfile(root,'runtime','evidence','observation_hybrid','conversion_values.mat'));
fixture=load(fullfile(root,'runtime','evidence','core_extraction','fixture.mat'));
config=g1.plannerConfiguration(root,'',false,formulation);
% conversion_values also contains 40 independent conversion-only test poses.
% Only the first fixture-sized segment belongs to the 4.8 s MPC replay.
states=golden.expected(1:size(fixture.raw,1),:);packets=fixture.ref;
if strcmp(formulation,'centroidal')
    % 同一套冻结运动学输入，转换到质心布局；两种执行器共享转换结果。
    % 参考重采样为所选 MPC 的发布时刻，测试不将它称为原始行走验收。
    [states,packets]=centroidalInputs(config,states,packets);
end
n=size(states,1);nx=size(states,2);nu=35;
old=zeros(n,nx+nu+2);direct=old;
g1_ocp_mex('init',config);oldCleanup=onCleanup(@()g1_ocp_mex('close'));
native=ocs2.Problem(config);newCleanup=onCleanup(@()delete(native)); %#ok<NASGU>
native.createSolver();native.disableGaitExtension();period=native.Dimensions(3);
requests=ceil(((0:floor(4.8/period+1e-12))*period-1e-12)/.002);
requests=unique(max(0,requests));
wall=tic;
expectedTargets=[3,1+nx+nu];expectedSchedules=[0,0];
for k=1:n
    tick=k-1;t=tick*.002;x=states(k,:)';request=ismember(tick,requests);
    % 旧协议参考包的 publish、tick 在 MATLAB 中设置，原生绑定不接收包。
    packet=packets(k,:)';packet(1)=tick;packet(2)=double(request);
    g1_ocp_mex('install_reference',packet);
    token=g1_ocp_mex('install_observation',x,[tick;t]);
    token=g1_ocp_mex('update_mpc',token,double(request));
    old(k,:)=g1_ocp_mex('query_policy',token)';

    native.setCurrentObservation(t,x,zeros(nu,1),3);
    if k==1
        initial=x;
        if strcmp(formulation,'whole_body')
            initial(end-28:end)=0;initial(5:6)=0;
        end
        native.reset(t,initial,zeros(nu,1));
    end
    if request
        [times,target,input,events,modes]=unpack(packet,nx,nu);
        native.setTargetTrajectories(times,target,input);
        native.setModeSchedule(events,modes);
        expectedTargets=expectedTargets+[1+2*numel(times),numel(times)*(1+nx+nu)];
        expectedSchedules=expectedSchedules+[2,numel(events)+numel(modes)];
        native.advanceMpc();assert(native.updatePolicy(),'No new native policy');
    end
    [xs,us,mode]=native.evaluatePolicy(t+.005,x);
    direct(k,:)=[xs;us;mode;t+.005]';
end
elapsed=toc(wall);counts=native.copyStats();
copyCountsExact=isequal(counts.targets,expectedTargets) && ...
    isequal(counts.schedules,expectedSchedules) && ...
    isequal(counts.observations,[2*n,n*(nx+nu)]) && ...
    isequal(counts.queryStates,[n,n*nx]) && ...
    isequal(counts.policyOutputs,[2*n,n*(nx+nu)]);
% 重复查询直接调用 MRT，不依赖旧 Controller 的 lastPolicy 缓存。
[xs,us,mode]=native.evaluatePolicy(t+.005,x);
duplicate=isequal([xs;us;mode;t+.005]',direct(end,:));
result=struct('formulation',formulation,'ticks',n,'updates',sum(ismember(0:n-1,requests)), ...
    'exact_equal',isequal(old,direct),'max_absolute_error',max(abs(old-direct),[],'all'), ...
    'repeated_query_exact',duplicate,'copy_counts_exact',copyCountsExact, ...
    'binding_copies',counts,'execution_wall_s',elapsed);
folder=fullfile(root,'runtime','evidence','ocs2_binding',formulation);
if ~isfolder(folder),mkdir(folder);end
save(fullfile(folder,'replay.mat'),'old','direct','result','-v7.3');
g1log.writeJson(fullfile(folder,'acceptance.json'),result);
clear oldCleanup;
end

function [times,target,input,events,modes]=unpack(a,nx,nu)
n=a(3);times=a(4:3+n);
target=reshape(a(7:6+3*nx),nx,3);target=target(:,1:n);
input=reshape(a(7+3*nx:6+3*(nx+nu)),nu,3);input=input(:,1:n);
offset=6+3*(nx+nu);count=a(offset+1);
% GaitSchedule 会移除末尾标记；适配现在明确放在 MATLAB。
events=[a(offset+2:offset+1+count);a(offset+131)];
modes=[a(offset+66:offset+66+count);3];
end

function [states,packets]=centroidalInputs(config,qv,raw)
g1_ocp_mex('init',config);cleanup=onCleanup(@()g1_ocp_mex('close')); %#ok<NASGU>
n=size(qv,1);states=zeros(n,35);packets=zeros(n,359);
% extractReference is already applied in fixture.ref (428 values per row).
% Preserve each row's current target knots; replace the domain-specific swing
% signature from centroidal task.info, rather than copying whole-body settings.
swing=[.05;0;.08;-.001;.4;-.15;.3;0];
latest=raw(1,:)';
for k=1:n
    states(k,:)=g1_ocp_mex('convert_observation',qv(k,:)')';
    if raw(k,2)~=0,latest=raw(k,:)';end
    a=latest;target=reshape(a(7:180),58,3);converted=zeros(35,3);
    for j=1:3,converted(:,j)=g1_ocp_mex('convert_observation',target(:,j));end
    packets(k,:)=[a(1:6);converted(:);a(181:285);a(286:416);swing;a(425:428)]';
end
end
