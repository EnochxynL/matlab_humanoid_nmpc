function result = validate_synchronous_backend()
%VALIDATE_SYNCHRONOUS_BACKEND 验证调度权属于外部，且失败后不会发布旧策略。
root = fileparts(fileparts(mfilename('fullpath')));
g1.initialize();
[kernels, blockCleanup] = planner_block_functions(); %#ok<ASGLU>
fixture = load(fullfile(root,'runtime','evidence','core_extraction','fixture.mat'));
golden = load(fullfile(root,'runtime','evidence','observation_hybrid','conversion_values.mat'));
folder = fullfile(root,'runtime','evidence','synchronous_backend','external_requests');
config = g1.plannerConfiguration(root,folder);
keys = {'TASK5A_AUTHORITY','TASK5A_PACKET_FILE'};
previous = {getenv(keys{1}),getenv(keys{2})};
cleanup = onCleanup(@()finish(keys,previous));
setenv(keys{1},'SIMULINK_REFERENCE'); setenv(keys{2},'IN_PROCESS_ONLY');

% 无效配置不能留下锁定实例。
assertInitRejected(struct());
bad = config; bad.taskFile = fullfile(folder,'nonexistent-task.info');
assertInitRejected(bad);
g1_ocp_mex('init',config);
reference = kernels.extractReference(fixture.ref(1,:)');
for tick = 0:10
    % 在 2 ms 主动额外求解；18 ms 到达传统截止时间时故意不求解。
    % 若 C++ 仍有自己的 60 Hz 调度，这两个行为会被拒绝或改变。
    requested = tick <= 1;
    if requested
        packet = reference; packet(1) = tick; packet(2) = 1;
        g1_ocp_mex('install_reference',packet);
    end
    identity = [tick;double(tick)*0.002;fixture.raw(tick+1,1)];
    token = g1_ocp_mex('install_observation',golden.expected(tick+1,:)',identity);
    completed = g1_ocp_mex('update_mpc',token,double(requested));
    duplicate = g1_ocp_mex('update_mpc',token,double(requested));
    assert(isequal(completed,duplicate));
    assertRejected(@()g1_ocp_mex('update_mpc',token,double(~requested)));
    policy = g1_ocp_mex('query_policy',completed);
    torque = g1_ocp_mex('inverse_dynamics',policy);
    assert(all(isfinite(torque)));
end

% 不安装匹配 tick 的参考而强制求解：同步抛错，后端进入失败状态。
tick = 11;
token = g1_ocp_mex('install_observation',golden.expected(tick+1,:)', ...
    [tick;double(tick)*0.002;fixture.raw(tick+1,1)]);
assertRejected(@()g1_ocp_mex('update_mpc',token,1));
assertRejected(@()g1_ocp_mex('update_mpc',token,0));
assertRejected(@()g1_ocp_mex('query_policy',token));
g1_ocp_mex('close');
solves = readtable(fullfile(folder,'solver_timing.csv'));
assert(height(solves)==2);
assert(isequal(solves.solver_target_control_time_s,[0;0.002]));
samples = readtable(fullfile(folder,'planner_samples.csv'));
assert(height(samples)==11);

% 关闭后允许重新创建、正常求解和重复关闭。
g1_ocp_mex('init',g1.plannerConfiguration(root,fullfile(folder,'reinitialized')));
g1_ocp_mex('install_reference',reference);
token = g1_ocp_mex('install_observation',golden.expected(1,:)',[0;0;fixture.raw(1,1)]);
completed = g1_ocp_mex('update_mpc',token,1);
policy = g1_ocp_mex('query_policy',completed);
torque = g1_ocp_mex('inverse_dynamics',policy); %#ok<NASGU>
g1_ocp_mex('close'); g1_ocp_mex('close');
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
    g1_ocp_mex('init',config);
catch exception
    rejected = strcmp(exception.identifier,'G1:PlannerMex');
end
assert(rejected,'Invalid backend configuration was accepted');
end

function finish(keys,previous)
g1_ocp_mex('close');
for k=1:numel(keys), setenv(keys{k},previous{k}); end
end
