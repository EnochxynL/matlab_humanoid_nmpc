function result = validate_planner_dataflow()
%VALIDATE_PLANNER_DATAFLOW Run the actual planner model on saved observations.
% Checks staged API guards, then the complete 2401-tick/289-update contract.
root = fileparts(fileparts(mfilename('fullpath')));
cd(root); g1.initialize();
folder = fullfile(root,'build','planner_dataflow','model_replay');
fixture = load(fullfile(root,'build','core_extraction','fixture.mat'));
keys = {'TASK5A_AUTHORITY','TASK5A_PACKET_FILE'};
old = {getenv(keys{1}),getenv(keys{2})};
setenv(keys{1},'SIMULINK_REFERENCE');setenv(keys{2},'IN_PROCESS_ONLY');
envCleanup = onCleanup(@()restoreEnvironment(keys,old)); %#ok<NASGU>
g1_ocp_mex('init',root,folder);
cleanup = onCleanup(@()g1_ocp_mex('close'));
g1_ocp_mex('reference',fixture.ref(1,:)');
token = g1_ocp_mex('prepare_observation',fixture.raw(1,:)');
assertRejected(@()g1_ocp_mex('query_policy',token));
assertRejected(@()g1_ocp_mex('update_mpc',token,0));
bad = token;bad(2) = bad(2)+0.002;
assertRejected(@()g1_ocp_mex('update_mpc',bad,1));
next = fixture.raw(2,:)';
assertRejected(@()g1_ocp_mex('prepare_observation',next));
clear cleanup;
% Fresh runtime and model persistent state; do not reuse guard-test state.
g1_ocp_mex('init',root,folder);
cleanup = onCleanup(@()g1_ocp_mex('close')); %#ok<NASGU>
ds = Simulink.SimulationData.Dataset;
t = (0:2400)'*0.002;
ds{1} = timeseries(fixture.raw,t);
ds{2} = timeseries(fixture.ref,t);
in = Simulink.SimulationInput('g1_planner');
in = in.setExternalInput(ds);
in = in.setModelParameter('StopTime','4.8','SaveOutput','on', ...
    'OutputSaveName','yout','SaveFormat','Dataset');
wall = tic;out = sim(in);execution = toc(wall);
clear cleanup;
samples = readtable(fullfile(folder,'planner_samples.csv'));
previous = readtable(fullfile(root,'build','directory_validation', ...
    'core_replay','planner_samples.csv'));
assert(isequal(samples{:,:},previous{:,:}),'Planner differs from accepted C++ replay.');
solves = readtable(fullfile(folder,'solver_timing.csv'));
assert(height(samples)==2401 && height(solves)==289);
actual = g1log.signalMatrix(out.yout{3}.Values);
assert(isequal(actual,samples{:,4:end}),'Model output differs from published action contract.');
identity = readtable(fullfile(folder,'time_identity.csv'));
assert(all(identity.canonical_control_time == double(identity.control_tick)*0.002));
assert(all(abs(identity.raw_minus_canonical)<=1e-12));
result = struct('passed',true,'ticks',height(samples),'updates',height(solves), ...
    'policy_action_exact_equal',true,'stage_guards_passed',true, ...
    'physical_simulation',false,'execution_wall_s',execution);
g1log.writeJson(fullfile(folder,'acceptance.json'),result);
save(fullfile(folder,'simulation_output.mat'),'out','result','-v7.3');
disp(result);
end
function assertRejected(callback)
rejected = false;
try
    ignored = callback(); %#ok<NASGU>
catch exception
    rejected = strcmp(exception.identifier,'G1:PlannerMex');
end
assert(rejected,'Illegal stage ordering/token was accepted.');
end
function restoreEnvironment(keys,values)
for k=1:numel(keys),setenv(keys{k},values{k});end
end
