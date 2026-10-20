function result = run_centroidal_query(stopTime, evidenceName)
%RUN_CENTROIDAL_QUERY 在独立 Simulink 模型中验证站立求解与策略查询。
% run_centroidal_query() 运行 50 ms；不驱动多体模型，不计算动作力矩。
% 模型外部输入为 observation74、reference359，后续可替换成实测数据。
if nargin < 1, stopTime = 0.05; end
if nargin < 2, evidenceName = 'centroidal_simulink'; end
assert(isscalar(stopTime) && isfinite(stopTime) && stopTime>=0 && ...
    abs(stopTime/0.002-round(stopTime/0.002))<1e-10);
cfg = g1.initialize();
cacheCleanup = g1.useBuildCache(); %#ok<NASGU>
root = cfg.projectRoot;
folder = fullfile(root,'runtime','evidence',evidenceName);
config = g1.plannerConfiguration(root,folder,true,'centroidal');
g1.plannerMex('init',config);
cleanup = onCleanup(@()g1.plannerMex('close'));
dimensions = g1.plannerMex('dimensions');
assert(isequal(dimensions,[35;35;23]));
assert(~mislocked('g1_inverse_dynamics_mex'),'Query-only mode initialized inverse dynamics.');

% 站立输入使用当前 G1 构型；此处零姿态四元数是明确的测试初值。
time = (0:round(stopTime/0.002))'*0.002;
raw = [time,repmat([cfg.initialPosition;1;0;0;0;zeros(6,1); ...
    cfg.initialQ;cfg.initialQd;1;1]',numel(time),1)];
q = [cfg.initialPosition;zeros(3,1);cfg.initialQ(cfg.mpcIndices)];
generalized = [q;zeros(29,1)];
state = g1.plannerMex('convert_observation',generalized);
assert(isequal(state,[zeros(6,1);q]));
moving = generalized; moving(30:32) = [.2;-.1;.05];
movingState = g1.plannerMex('convert_observation',moving);
assert(max(abs(movingState(1:6)-[.2;-.1;.05;zeros(3,1)]))<1e-12);

% 显式参考，不套用全身目标布局。零动量、标称姿态、零关节速度参考。
% 该站立时间表覆盖测试窗口；不额外生成末端摆动或关节力矩。
reference = zeros(numel(time),359);
reference(:,1) = (0:numel(time)-1)';
expectedTicks = unique(ceil(((0:floor(time(end)*80))/80-1e-12)/0.002));
expectedTicks(expectedTicks<0)=0;
reference(expectedTicks+1,2) = 1;
reference(:,3) = 1;
reference(:,4) = time;
reference(:,7:41) = repmat(state',numel(time),1);
reference(:,217) = 3;
reference(:,218:220) = repmat([-.5,.5,stopTime+2.5],numel(time),1);
reference(:,282:285) = 3;
reference(:,347) = stopTime+10;
reference(:,348:355) = repmat([.05,0,.08,-.001,.4,-.15,.3,0],numel(time),1);

open_system(fullfile(root,'models','g1_centroidal_planner.slx'));
ds = Simulink.SimulationData.Dataset;
ds{1} = timeseries(raw,time);
ds{2} = timeseries(reference,time);
in = Simulink.SimulationInput('g1_centroidal_planner');
in = in.setExternalInput(ds);
in = in.setModelParameter('StopTime',num2str(stopTime,17), ...
    'SaveOutput','on','OutputSaveName','yout','SaveFormat','Dataset');
wall = tic;
out = sim(in);
elapsed = toc(wall);
last = g1.plannerMex('query_policy',[numel(time)-1;time(end);1/80]);
duplicate = g1.plannerMex('query_policy',[numel(time)-1;time(end);1/80]);
assert(isequal(last,duplicate),'Duplicate centroidal query changed the policy.');
clear cleanup;

policy = g1log.signalMatrix(out.yout{1}.Values);
observed = g1log.signalMatrix(out.yout{2}.Values);
assert(isequal(size(policy),[numel(time),72]) && all(isfinite(policy),'all'));
assert(isequal(policy(end,:)',last));
assert(isequal(observed,repmat(state',numel(time),1)));
assert(all(policy(:,71)==3));
assert(all(abs(policy(:,72)-(time+0.005))<1e-12));
solves = readtable(fullfile(folder,'solver_timing.csv'));
assert(isequal(solves.solver_target_control_time_s,time(expectedTicks+1)));
identity = readtable(fullfile(folder,'time_identity.csv'));
assert(height(identity)==numel(time));
result = struct('passed',true,'ticks',numel(time),'updates',height(solves), ...
    'state_dimension',35,'input_dimension',35,'policy_dimension',72, ...
    'duplicate_query_exact',true,'inverse_dynamics_initialized',false, ...
    'physical_simulation',false,'execution_wall_s',elapsed);
g1log.writeJson(fullfile(folder,'acceptance.json'),result);
save(fullfile(folder,'simulation_output.mat'),'out','result','-v7.3');
disp(result);
end
