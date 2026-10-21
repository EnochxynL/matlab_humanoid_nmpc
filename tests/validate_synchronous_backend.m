function result=validate_synchronous_backend()
% 验证一次显式控制步、外部求解请求和重复调用保护。
root=fileparts(fileparts(mfilename('fullpath')));g1.initialize();
[kernels,blockCleanup]=planner_block_functions(); %#ok<ASGLU>
fixture=load(fullfile(root,'runtime','evidence','core_extraction','fixture.mat'));
golden=load(fullfile(root,'runtime','evidence','observation_hybrid','conversion_values.mat'));
folder=fullfile(root,'runtime','evidence','synchronous_backend','external_requests');
config=g1.plannerConfiguration(root,folder);cleanup=onCleanup(@()g1.plannerMex('close'));
assertInitRejected(struct());
bad=config;bad.taskFile=fullfile(folder,'nonexistent-task.info');assertInitRejected(bad);
g1.plannerMex('init',config);
state=golden.expected(1,:)';reference=kernels.extractReference(fixture.ref(1,:)');
assertRejected(@()g1.plannerMex('step',state,reference,[1;.002;.002],1));
assertRejected(@()g1.plannerMex('step',state,reference,[0;eps;0],1));
assertRejected(@()g1.plannerMex('step',state,reference,[0;0;1e-6],1));
assertRejected(@()g1.plannerMex('step',state,reference,[0;0;0],0));
for tick=0:10
    requested=double(tick<=1);packet=reference;packet(1)=tick;packet(2)=requested;
    identity=[tick;tick*.002;fixture.raw(tick+1,1)];state=golden.expected(tick+1,:)';
    policy=g1.plannerMex('step',state,packet,identity,requested);
    assert(isequal(policy,g1.plannerMex('step',state,packet,identity,requested)));
    assertRejected(@()g1.plannerMex('step',state,packet,identity,1-requested));
    torque=g1.plannerMex('inverse_dynamics',policy);assert(all(isfinite(torque)));
end
assertRejected(@()g1.plannerMex('step',state,packet,[12;.024;.024],0));
% 无效参考在提交求解前拒绝；不把可修正的输入错误当作求解器故障。
assertRejected(@()g1.plannerMex('step',state,reference,[11;.022;.022],1));
g1.plannerMex('close');
solves=readtable(fullfile(folder,'solver_timing.csv'));
assert(isequal(solves.solver_target_control_time_s,[0;.002]));
samples=readtable(fullfile(folder,'planner_samples.csv'));assert(height(samples)==11);
timeIdentity=readtable(fullfile(folder,'time_identity.csv'));
assert(isequal(timeIdentity.control_tick,(0:10)'));
assert(all(timeIdentity.canonical_control_time==timeIdentity.control_tick*.002));
trace=readtable(fullfile(folder,'timing_trace.csv'));
assert(sum(strcmp(trace.event,'solver_finished'))==2);
assert(sum(strcmp(trace.event,'policy_consumed'))==11);
assert(all(isfinite(trace.wall_monotonic_time_s)) && all(diff(trace.wall_monotonic_time_s)>=0));
g1.plannerMex('init',g1.plannerConfiguration(root,fullfile(folder,'reinitialized')));
policy=g1.plannerMex('step',golden.expected(1,:)',reference,[0;0;fixture.raw(1,1)],1);
torque=g1.plannerMex('inverse_dynamics',policy); %#ok<NASGU>
g1.plannerMex('close');g1.plannerMex('close');
assert(~mislocked('ocs2_binding_mex'));
result=struct('passed',true,'external_request_times',[0;.002],'ticks',11, ...
    'duplicate_requests_idempotent',true,'invalid_reference_rejected',true,'reinitialization_passed',true);
g1log.writeJson(fullfile(folder,'acceptance.json'),result);disp(result);
end

function assertRejected(callback)
rejected = false;
try
    ignored = callback(); %#ok<NASGU>
catch exception
    rejected = strcmp(exception.identifier,'G1:PlannerMex');
end
assert(rejected,'Invalid backend operation was accepted');
end

function assertInitRejected(config)
rejected = false;
try
    g1.plannerMex('init',config);
catch exception
    rejected = strcmp(exception.identifier,'G1:PlannerMex');
end
assert(rejected,'Invalid backend configuration was accepted');
end
