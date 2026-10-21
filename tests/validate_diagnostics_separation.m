function result = validate_diagnostics_separation()
%VALIDATE_DIAGNOSTICS_SEPARATION 关闭诊断仍运行真实规划模型，并保留入口检查。
root = fileparts(fileparts(mfilename('fullpath')));
g1.initialize();
cacheCleanup = g1.useBuildCache(); %#ok<NASGU>
[kernels, blockCleanup] = planner_block_functions(); %#ok<ASGLU>
fixture = load(fullfile(root,'runtime','evidence','core_extraction','fixture.mat'));
golden = load(fullfile(root,'runtime','evidence','observation_hybrid','conversion_values.mat'));
evidence = fullfile(root,'runtime','evidence','diagnostic_separation');
unusedFolder = tempname(fullfile(root,'native','build'));
config = g1.plannerConfiguration(root,unusedFolder,false);
cleanup = onCleanup(@()g1.plannerMex('close'));

g1.plannerMex('init',config);
assert(~isfolder(unusedFolder),'Disabled diagnostics created a log directory');
reference=kernels.extractReference(fixture.ref(1,:)');
state=golden.expected(1,:)';identity=[0;0;fixture.raw(1,1)];
assertRejected(@()g1.plannerMex('step',state,reference,[0;0;.1],1));
assertRejected(@()g1.plannerMex('step',state,reference,identity,0));
policy=g1.plannerMex('step',state,reference,identity,1);
bad=policy;bad(1)=bad(1)+.01;
assertRejected(@()g1.plannerMex('inverse_dynamics',bad));
torque=g1.plannerMex('inverse_dynamics',policy);
assert(isequal(torque,g1.plannerMex('inverse_dynamics',policy)));
assert(isequal(policy,g1.plannerMex('step',state,reference,identity,1)));
assertRejected(@()g1.plannerMex('step',golden.expected(3,:)',reference,[2;.004;.004],0));

g1.plannerMex('close');

% 新实例和新的模型仿真状态，完整执行 2401 个控制 tick。
g1.plannerMex('init',config);
ds = Simulink.SimulationData.Dataset;
t = (0:2400)'*0.002;
ds{1} = timeseries(fixture.raw,t);
ds{2} = timeseries(fixture.ref,t);
in = Simulink.SimulationInput('g1_planner');
in = in.setExternalInput(ds);
in = in.setModelParameter('StopTime','4.8','SaveOutput','on', ...
    'OutputSaveName','yout','SaveFormat','Dataset');
out = sim(in);
g1.plannerMex('close');
assert(~isfolder(unusedFolder),'Disabled diagnostics wrote application log files');
actual = g1log.signalMatrix(out.yout{3}.Values);
enabled = readtable(fullfile(evidence,'enabled','model_replay','planner_samples.csv'));
expected = enabled{:,4:end};
assert(isequal(size(actual),size(expected)) && size(actual,1)==2401);
assert(all(isfinite(actual),'all'));
assert(isequal(actual(:,117:118),expected(:,117:118)),'Mode/query time changed');
result = struct('passed',true,'ticks',2401,'runtime_guards_passed',true, ...
    'diagnostic_directory_created',false,'outputs_exact_equal',isequal(actual,expected), ...
    'maximum_absolute_error',max(abs(actual-expected),[],'all'));
g1log.writeJson(fullfile(evidence,'disabled_acceptance.json'),result);
save(fullfile(evidence,'disabled_output.mat'),'out','result','-v7.3');
disp(result);
end

function assertRejected(callback)
rejected = false;
try
    ignored = callback(); %#ok<NASGU>
catch exception
    rejected = strcmp(exception.identifier,'G1:PlannerMex');
end
assert(rejected,'Runtime guard was disabled with diagnostics');
end
