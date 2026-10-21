classdef NativeDiagnosticsTest < matlab.unittest.TestCase
    % 已保存的迁移前表格用于检查数值、行序和未知值语义。
    properties (TestParameter)
        formulation={'whole_body','centroidal'}
    end
    methods (TestClassSetup)
        function paths(~)
            root=fileparts(fileparts(mfilename('fullpath')));
            addpath(fullfile(root,'matlab'),fullfile(root,'tools'),fullfile(root,'tests'));
            g1.initialize();
        end
    end
    methods (Test)
        function nativeComputedTables(testCase,formulation)
            result=checkPolicyDiagnostics(formulation);
            testCase.verifyTrue(result.exactPolicyTables);
            testCase.verifyTrue(result.solverStatisticsRoundoffOnly);
            testCase.verifyTrue(result.exactConfiguration);
            testCase.verifyTrue(result.nativeSchemasPresent);
            testCase.verifyTrue(result.finiteNativeClock);
        end
        function configurationMetadata(testCase,formulation)
            result=readConfigurationMetadata(formulation);
            testCase.verifyEqual(result.nativeJson,result.baselineJson);
            testCase.verifyEqual(result.backendJson,result.baselineJson);
        end
    end
end

function result=readConfigurationMetadata(formulation)
root=fileparts(fileparts(mfilename('fullpath')));
config=g1.plannerConfiguration(root,'',true,formulation);
problem=ocs2.Problem(config);cleanup=onCleanup(@()delete(problem)); %#ok<NASGU>
problem.createSolver();problem.enableDiagnostics();batch=problem.diagnostics_drain();
result.nativeJson=batch.configurationJson;
backend=g1.PlannerBackend(config);backendCleanup=onCleanup(@()delete(backend)); %#ok<NASGU>
batch=backend.execute('diagnostics');result.backendJson=batch.configurationJson;
result.baselineJson=fileread(fullfile(root,'runtime','evidence', ...
    'diagnostic_policy_baseline',formulation,'loaded_configuration.json'));
end

function result=checkPolicyDiagnostics(formulation)
root=fileparts(fileparts(mfilename('fullpath')));
previousDirectory=pwd;directoryCleanup=onCleanup(@()cd(previousDirectory)); %#ok<NASGU>
cd(root);
if strcmp(formulation,'whole_body')
    validate_binding_migration();
    after=fullfile(root,'runtime','evidence','binding_migration','model_replay');
else
    validate_centroidal_binding_migration();
    after=fullfile(root,'runtime','evidence','binding_migration','centroidal');
end
before=fullfile(root,'runtime','evidence','diagnostic_policy_baseline',formulation);
names={'policy_knots','gait_schedule','policy_vectors','policy_queries','structural_solver'};
statisticsError=0;
for k=1:numel(names)
    expected=readtable(fullfile(before,[names{k} '.csv']));
    actual=readtable(fullfile(after,[names{k} '.csv']));
    if strcmp(names{k},'policy_queries')
        assert(all(isfinite(actual.wall_query_time_s)));
        assert(all(diff(actual.wall_query_time_s)>=0));
        expected.wall_query_time_s=[];actual.wall_query_time_s=[];
    end
    if strcmp(names{k},'structural_solver')
        % 同一数值解的并行归约统计可能相差数个 ULP；不放宽离散列或未知值检查。
        expectedValues=expected{:,5:8};actualValues=actual{:,5:8};
        difference=abs(expectedValues-actualValues);
        assert(all(difference<=64*eps(max(1,abs(expectedValues))),'all'));
        statisticsError=max(difference,[],'all');
        expected(:,5:8)=[];actual(:,5:8)=[];
    end
    assert(isequaln(expected,actual),'Changed table: %s / %s',formulation,names{k});
end
oldConfiguration=jsondecode(fileread(fullfile(before,'loaded_configuration.json')));
configuration=jsondecode(fileread(fullfile(after,'loaded_configuration.json')));
assert(isequaln(oldConfiguration,configuration),'Loaded settings changed');

config=g1.plannerConfiguration(root,'',false,formulation);
problem=ocs2.Problem(config);cleanup=onCleanup(@()delete(problem)); %#ok<NASGU>
problem.createSolver();problem.enableDiagnostics();batch=problem.diagnostics_drain();
assert(all(ismember(names,{batch.tables.name})));
assert(strcmp(batch.configurationJson,fileread(fullfile(before,'loaded_configuration.json'))));
% Application-owned tables must not be recreated by native diagnostics.
assert(~any(ismember({'reference','gait_queries','timing_trace','logging_load','adapter_consumption'}, ...
    {batch.tables.name})));
result=struct('exactPolicyTables',true,'solverStatisticsRoundoffOnly',true, ...
    'solverStatisticsMaxAbsoluteError',statisticsError,'exactConfiguration',true, ...
    'nativeSchemasPresent',true,'finiteNativeClock',true);
g1log.writeJson(fullfile(after,'native_diagnostics_acceptance.json'),result);
end
