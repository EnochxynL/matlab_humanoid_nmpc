function result = validate_matlab_diagnostics()
%VALIDATE_MATLAB_DIAGNOSTICS Native batches have no file ownership or replay side effects.
root = fileparts(fileparts(mfilename('fullpath')));
g1.initialize();
[kernels,blockCleanup] = planner_block_functions(); %#ok<ASGLU>
fixture = load(fullfile(root,'runtime','evidence','core_extraction','fixture.mat'));
golden = load(fullfile(root,'runtime','evidence','observation_hybrid','conversion_values.mat'));
cleanup = onCleanup(@closeNative);
unusedFolder = tempname(fullfile(root,'native','build'));
config = g1.plannerConfiguration(root,unusedFolder);
backend = g1.PlannerBackend(config);
backendCleanup = onCleanup(@()delete(backend));
initial = backend.execute('diagnostics');
assert(~isfolder(unusedFolder),'Native diagnostics created a log folder');
assert(~isempty(initial.configurationJson) && ~isempty(initial.tables));
info = jsondecode(initial.configurationJson);
assert(str2double(string(info.state_dim))==58);
assertEmpty(backend.execute('diagnostics'));

policy = backend.execute('step',golden.expected(1,:)', ...
    kernels.extractReference(fixture.ref(1,:)'),[0;0],1);
g1_inverse_dynamics_mex('init',config);
torque = g1_inverse_dynamics_mex('compute',policy(1:58),policy(59:93));
batch = backend.execute('diagnostics');
names = {batch.tables.name};
assert(~any(strcmp(names,'time_identity')));
assert(~any(strcmp(names,'planner_samples')));
assert(size(batch.tables(strcmp(names,'solver_timing')).values,1)==1);
assert(numel(batch.snapshot)==218);
assertEmpty(backend.execute('diagnostics'));
delete(backend); clear backendCleanup;
g1_inverse_dynamics_mex('close');
assert(~isfolder(unusedFolder),'Native diagnostic computation wrote files');

folder = fullfile(root,'runtime','evidence','matlab_diagnostics','raw_export');
writer = g1.DiagnosticWriter(folder);
writer.append(initial); writer.append(batch); writer.flush();
writer.sample(0,fixture.raw(1,1),0,[policy(1:93);torque;policy(94:95)]);
writer.flush();
samples = readtable(fullfile(folder,'planner_samples.csv'));
assert(height(samples)==1 && samples.control_tick==0);
assert(isequal(samples{1,4:61},policy(1:58)'));
fid = fopen(fullfile(folder,'structural_snapshot.bin'),'r');
closeSnapshot = onCleanup(@()fclose(fid));
assert(isequaln(fread(fid,'double'),batch.snapshot));
clear closeSnapshot;
delete(writer);

% The disabled path has no schema, metadata, snapshot, or rows.
config.diagnosticsEnabled = false;
backend = g1.PlannerBackend(config);
backendCleanup = onCleanup(@()delete(backend));
disabled = backend.execute('diagnostics');
assertEmpty(disabled); assert(isempty(disabled.tables));
delete(backend); clear backendCleanup;
assert(~isfolder(unusedFolder));

% Writer initialization failure must release the already created native session.
blocked = fullfile(folder,'blocked_directory');
fid = fopen(blocked,'w'); fclose(fid);
config.diagnosticsEnabled = true; config.logDirectory = blocked;
rejected = false;
try
    g1.plannerMex('init',config);
catch
    rejected = true;
end
assert(rejected && ~mislocked('ocs2_binding_mex') && ~mislocked('g1_inverse_dynamics_mex'));
g1.plannerMex('init',g1.plannerConfiguration(root,unusedFolder,false));
g1.plannerMex('close');
result = struct('passed',true,'native_file_output',false, ...
    'drain_idempotent',true,'snapshot_values',218, ...
    'disabled_batch_empty',true,'writer_failure_releases_session',true);
g1log.writeJson(fullfile(folder,'acceptance.json'),result);
disp(result);
end

function closeNative()
g1.plannerMex('close');
g1_inverse_dynamics_mex('close');
end

function assertEmpty(batch)
assert(isempty(batch.snapshot) && isempty(batch.configurationJson) && isempty(batch.loadedLibraries));
for k=1:numel(batch.tables), assert(isempty(batch.tables(k).values)); end
end
