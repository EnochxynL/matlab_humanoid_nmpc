function result=validate_matlab_diagnostic_tables()
% Compare migrated rows against accepted pre-migration data, excluding wall clocks.
root=fileparts(fileparts(mfilename('fullpath')));g1.initialize();
result.stage=validate_synchronous_backend();
result.replay=validate_planner_dataflow('matlab_diagnostic_tables',true);
before=fullfile(root,'runtime','evidence','shared_walking','model_replay');
after=fullfile(root,'runtime','evidence','matlab_diagnostic_tables','model_replay');
names={'reference','gait_queries','adapter_consumption','planner_samples','time_identity'};
for k=1:numel(names)
    old=readtable(fullfile(before,[names{k} '.csv']));
    actual=readtable(fullfile(after,[names{k} '.csv']));
    assert(isequal(old,actual),'Changed diagnostic table: %s',names{k});
end
old=readtable(fullfile(before,'timing_trace.csv'));
actual=readtable(fullfile(after,'timing_trace.csv'));
assert(isequal(old(:,1:2),actual(:,1:2)),'Diagnostic event identity/order changed');
assert(all(isfinite(actual.wall_monotonic_time_s)) && all(diff(actual.wall_monotonic_time_s)>=0));
assert(height(readtable(fullfile(after,'logging_load.csv')))==0);
clock=jsondecode(fileread(fullfile(after,'diagnostic_clock.json')));
assert(strcmp(clock.clock,'MATLAB tic/toc'));

% Native binding must no longer define or populate MATLAB-owned tables.
config=g1.plannerConfiguration(root,'',false);
native=ocs2.Problem(config);cleanup=onCleanup(@()delete(native)); %#ok<NASGU>
native.createSolver();native.enableDiagnostics();batch=native.diagnostics_drain();
assert(~any(ismember({batch.tables.name},{'reference','gait_queries','timing_trace','logging_load','adapter_consumption'})));
result.exact_table_values=true;result.exact_trace_events=true;
result.native_local_tables_absent=true;result.clock='MATLAB session relative';
g1log.writeJson(fullfile(root,'runtime','evidence','matlab_diagnostic_tables','acceptance.json'),result);
end
