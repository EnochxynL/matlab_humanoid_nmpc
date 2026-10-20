function result = validate_synchronous_backend()
%VALIDATE_SYNCHRONOUS_BACKEND 验证调度权属于外部，且失败后不会发布旧策略。
root = fileparts(fileparts(mfilename('fullpath')));
g1.initialize();
[kernels, blockCleanup] = planner_block_functions(); %#ok<ASGLU>
fixture = load(fullfile(root,'runtime','evidence','core_extraction','fixture.mat'));
golden = load(fullfile(root,'runtime','evidence','observation_hybrid','conversion_values.mat'));
folder = fullfile(root,'runtime','evidence','synchronous_backend','external_requests');
config = g1.plannerConfiguration(root,folder);
cleanup = onCleanup(@()g1.plannerMex('close'));

% 无效配置不能留下锁定实例。
assertInitRejected(struct());
bad = config; bad.taskFile = fullfile(folder,'nonexistent-task.info');
assertInitRejected(bad);
g1.plannerMex('init',config);
reference = kernels.extractReference(fixture.ref(1,:)');
for tick = 0:10
    % 在 2 ms 主动额外求解；18 ms 到达传统截止时间时故意不求解。
    % 若 C++ 仍有自己的 60 Hz 调度，这两个行为会被拒绝或改变。
    requested = tick <= 1;
    if requested
        packet = reference; packet(1) = tick; packet(2) = 1;
        g1.plannerMex('install_reference',packet);
    end
    identity = [tick;double(tick)*0.002;fixture.raw(tick+1,1)];
    token = g1.plannerMex('install_observation',golden.expected(tick+1,:)',identity);
    completed = g1.plannerMex('update_mpc',token,double(requested));
    duplicate = g1.plannerMex('update_mpc',token,double(requested));
    assert(isequal(completed,duplicate));
    assertRejected(@()g1.plannerMex('update_mpc',token,double(~requested)));
    policy = g1.plannerMex('query_policy',completed);
    torque = g1.plannerMex('inverse_dynamics',policy);
    assert(all(isfinite(torque)));
end

% 不安装匹配 tick 的参考而强制求解：同步抛错，后端进入失败状态。
tick = 11;
token = g1.plannerMex('install_observation',golden.expected(tick+1,:)', ...
    [tick;double(tick)*0.002;fixture.raw(tick+1,1)]);
assertRejected(@()g1.plannerMex('update_mpc',token,1));
assertRejected(@()g1.plannerMex('update_mpc',token,0));
assertRejected(@()g1.plannerMex('query_policy',token));
g1.plannerMex('close');
solves = readtable(fullfile(folder,'solver_timing.csv'));
assert(height(solves)==2);
assert(isequal(solves.solver_target_control_time_s,[0;0.002]));
samples = readtable(fullfile(folder,'planner_samples.csv'));
assert(height(samples)==11);
trace = readtable(fullfile(folder,'timing_trace.csv'));
assert(isequal(trace.Properties.VariableNames, ...
    {'event','control_time_s','wall_monotonic_time_s'}));
assert(all(ismember(trace.event,{'observation_arrived','solver_started', ...
    'solver_finished','policy_installed','policy_consumed'})));
assert(sum(strcmp(trace.event,'solver_finished'))==2);
assert(sum(strcmp(trace.event,'policy_installed'))==2);
assert(sum(strcmp(trace.event,'policy_consumed'))==11);

% 关闭后允许重新创建、正常求解和重复关闭。
g1.plannerMex('init',g1.plannerConfiguration(root,fullfile(folder,'reinitialized')));
g1.plannerMex('install_reference',reference);
token = g1.plannerMex('install_observation',golden.expected(1,:)',[0;0;fixture.raw(1,1)]);
completed = g1.plannerMex('update_mpc',token,1);
policy = g1.plannerMex('query_policy',completed);
torque = g1.plannerMex('inverse_dynamics',policy); %#ok<NASGU>
g1.plannerMex('close'); g1.plannerMex('close');
assert(~mislocked('g1_ocp_mex'));
result = struct('passed',true,'external_request_times',[0;0.002], ...
    'ticks',11,'duplicate_requests_idempotent',true, ...
    'failed_backend_rejected',true,'reinitialization_passed',true);
g1log.writeJson(fullfile(folder,'acceptance.json'),result);
disp(result);
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
