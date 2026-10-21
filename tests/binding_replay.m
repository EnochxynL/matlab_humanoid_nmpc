function result=binding_replay(formulation)
%BINDING_REPLAY 用冻结旧控制器输出验证直接原生绑定。
% 不做物理仿真。所有调度、reset 目标及查询提前量明确写在 MATLAB。
root=fileparts(fileparts(mfilename('fullpath')));g1.initialize();
[states,packets,config]=binding_replay_inputs(formulation);
baseline=load(fullfile(root,'tests','fixtures','legacy_controller',[formulation '.mat']),'old');
old=baseline.old;
n=size(states,1);nx=size(states,2);nu=35;
direct=zeros(n,nx+nu+2);
assert(isequal(size(old),size(direct)),'Baseline dimensions changed');
native=ocs2.Problem(config);newCleanup=onCleanup(@()delete(native)); %#ok<NASGU>
native.createSolver();native.reference_getGaitSchedule_disableExtension();period=native.Dimensions(3);
requests=ceil(((0:floor(4.8/period+1e-12))*period-1e-12)/.002);
requests=unique(max(0,requests));
wall=tic;
expectedTargets=[3,1+nx+nu];expectedSchedules=[0,0];
for k=1:n
    tick=k-1;t=tick*.002;x=states(k,:)';request=ismember(tick,requests);
    % 旧协议参考包的 publish、tick 在 MATLAB 中设置，原生绑定不接收包。
    packet=packets(k,:)';packet(1)=tick;packet(2)=double(request);

    native.mrt_setCurrentObservation(t,x,zeros(nu,1),3);
    if k==1
        initial=x;
        if strcmp(formulation,'whole_body')
            initial(end-28:end)=0;initial(5:6)=0;
        end
        native.mrt_resetMpcNode(t,initial,zeros(nu,1));
    end
    if request
        [times,target,input,events,modes]=unpack(packet,nx,nu);
        native.reference_setTargetTrajectories(times,target,input);
        native.reference_getGaitSchedule_updateModeSchedule(events,modes);
        expectedTargets=expectedTargets+[1+2*numel(times),numel(times)*(1+nx+nu)];
        expectedSchedules=expectedSchedules+[2,numel(events)+numel(modes)];
        native.mrt_advanceMpc();assert(native.mrt_updatePolicy(),'No new native policy');
    end
    [xs,us,mode]=native.mrt_evaluatePolicy(t+.005,x);
    direct(k,:)=[xs;us;mode;t+.005]';
end
elapsed=toc(wall);counts=native.copyStats();
copyCountsExact=isequal(counts.targets,expectedTargets) && ...
    isequal(counts.schedules,expectedSchedules) && ...
    isequal(counts.observations,[2*n,n*(nx+nu)]) && ...
    isequal(counts.queryStates,[n,n*nx]) && ...
    isequal(counts.policyOutputs,[2*n,n*(nx+nu)]);
% 重复查询直接调用 MRT，不依赖旧 Controller 的 lastPolicy 缓存。
[xs,us,mode]=native.mrt_evaluatePolicy(t+.005,x);
duplicate=isequal([xs;us;mode;t+.005]',direct(end,:));
result=struct('formulation',formulation,'ticks',n,'updates',sum(ismember(0:n-1,requests)), ...
    'exact_equal',isequal(old,direct),'max_absolute_error',max(abs(old-direct),[],'all'), ...
    'repeated_query_exact',duplicate,'copy_counts_exact',copyCountsExact, ...
    'binding_copies',counts,'execution_wall_s',elapsed);
folder=fullfile(root,'runtime','evidence','ocs2_binding',formulation);
if ~isfolder(folder),mkdir(folder);end
save(fullfile(folder,'replay.mat'),'direct','result','-v7.3');
g1log.writeJson(fullfile(folder,'acceptance.json'),result);
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
