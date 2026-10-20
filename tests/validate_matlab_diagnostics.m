function result = validate_matlab_diagnostics()
%VALIDATE_MATLAB_DIAGNOSTICS Native batches have no file ownership or replay side effects.
root = fileparts(fileparts(mfilename('fullpath')));
g1.initialize();
[kernels,blockCleanup] = planner_block_functions(); %#ok<ASGLU>
fixture = load(fullfile(root,'runtime','evidence','core_extraction','fixture.mat'));
golden = load(fullfile(root,'runtime','evidence','observation_hybrid','conversion_values.mat'));
cleanup = onCleanup(@()g1_ocp_mex('close'));
unusedFolder = tempname(fullfile(root,'native','build'));
config = g1.plannerConfiguration(root,unusedFolder);
g1_ocp_mex('init',config);
initial = g1_ocp_mex('diagnostics');
assert(~isfolder(unusedFolder),'Native diagnostics created a log folder');
assert(~isempty(initial.configurationJson) && ~isempty(initial.tables));
info = jsondecode(initial.configurationJson);
assert(str2double(string(info.state_dim))==58);
assertEmpty(g1_ocp_mex('diagnostics'));

g1_ocp_mex('install_reference',kernels.extractReference(fixture.ref(1,:)'));
token = g1_ocp_mex('install_observation',golden.expected(1,:)',[0;0;fixture.raw(1,1)]);
completed = g1_ocp_mex('update_mpc',token,1);
policy = g1_ocp_mex('query_policy',completed);
torque = g1_ocp_mex('inverse_dynamics',policy); %#ok<NASGU>
batch = g1_ocp_mex('diagnostics');
names = {batch.tables.name};
assert(size(batch.tables(strcmp(names,'planner_samples')).values,1)==1);
assert(size(batch.tables(strcmp(names,'solver_timing')).values,1)==1);
assert(numel(batch.snapshot)==218);
assertEmpty(g1_ocp_mex('diagnostics'));
g1_ocp_mex('close');
assert(~isfolder(unusedFolder),'Native diagnostic computation wrote files');

folder = fullfile(root,'runtime','evidence','matlab_diagnostics','raw_export');
writer = g1.DiagnosticWriter(folder);
writer.append(initial); writer.append(batch); writer.flush();
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
g1_ocp_mex('init',config);
disabled = g1_ocp_mex('diagnostics');
assertEmpty(disabled); assert(isempty(disabled.tables));
g1_ocp_mex('close');
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
assert(rejected && ~mislocked('g1_ocp_mex'));
g1.plannerMex('init',g1.plannerConfiguration(root,unusedFolder,false));
g1.plannerMex('close');
result = struct('passed',true,'native_file_output',false, ...
    'drain_idempotent',true,'snapshot_values',218, ...
    'disabled_batch_empty',true,'writer_failure_releases_session',true);
g1log.writeJson(fullfile(folder,'acceptance.json'),result);
disp(result);
end

function assertEmpty(batch)
assert(isempty(batch.snapshot) && isempty(batch.configurationJson) && isempty(batch.loadedLibraries));
for k=1:numel(batch.tables), assert(isempty(batch.tables(k).values)); end
end
