function result = validate_observation_split(evidenceName)
[kernels,blockCleanup]=planner_block_functions();
if nargin==0,evidenceName='observation_split';end
%VALIDATE_OBSERVATION_SPLIT Independent extraction, install guards and full replay.
root=fileparts(fileparts(mfilename('fullpath')));cd(root);cfg=g1.initialize();
folder=fullfile(root,'runtime','evidence',evidenceName);
fixture=load(fullfile(root,'runtime','evidence','core_extraction','fixture.mat'));
golden=load(fullfile(root,'runtime','evidence','observation_hybrid','conversion_values.mat'));
indices=double(cfg.mpcIndices(:)');
% Extraction must work before controller initialization, across the complete sequence.
for j=1:2401
    identity=kernels.extractObservation(fixture.raw(j,:)',golden.expected(j,:)',indices);
    assert(isequal(identity,[double(j-1);double(j-1)*.002;fixture.raw(j,1)]));
end
raw=fixture.raw(1,:)';state=golden.expected(1,:)';
bad=raw;bad(4)=.2;reject(@()kernels.extractObservation(bad,state,indices),'G1:ObservationExtraction');
bad=raw;bad(1)=.001;reject(@()kernels.extractObservation(bad,state,indices),'G1:ObservationExtraction');
bad=raw;bad(5:8)=0;reject(@()kernels.extractObservation(bad,state,indices),'G1:ObservationExtraction');
bad=raw;bad(9)=NaN;reject(@()kernels.extractObservation(bad,state,indices),'G1:ObservationExtraction');
bad=state;bad(7)=bad(7)+1;reject(@()kernels.extractObservation(raw,bad,indices),'G1:ObservationExtraction');
bad=indices;bad(2)=bad(1);reject(@()kernels.extractObservation(raw,state,bad),'G1:ObservationExtraction');
keys={'TASK5A_AUTHORITY','TASK5A_PACKET_FILE'};old={getenv(keys{1}),getenv(keys{2})};
envCleanup=onCleanup(@()restoreEnvironment(keys,old)); %#ok<NASGU>
setenv(keys{1},'SIMULINK_REFERENCE');setenv(keys{2},'IN_PROCESS_ONLY');
g1_ocp_mex('init',g1.plannerConfiguration(root,fullfile(folder,'install_guards')));
cleanup=onCleanup(@()g1_ocp_mex('close'));
identity=kernels.extractObservation(raw,state,indices);
bad=identity;bad(2)=.001;reject(@()g1_ocp_mex('install_observation',state,bad),'G1:PlannerMex');
token=g1_ocp_mex('install_observation',state,identity);
reject(@()g1_ocp_mex('query_policy',token),'G1:PlannerMex');
nextIdentity=kernels.extractObservation(fixture.raw(2,:)',golden.expected(2,:)',indices);
reject(@()g1_ocp_mex('install_observation',golden.expected(2,:)',nextIdentity),'G1:PlannerMex');
clear cleanup;
replay=validate_planner_dataflow(evidenceName,true);
result=struct('passed',true,'independent_extraction_ticks',2401,'negative_cases',8,'replay',replay);
g1log.writeJson(fullfile(folder,'acceptance.json'),result);disp(result);
end
function reject(callback,id)
rejected=false;
try
    ignored=callback(); %#ok<NASGU>
catch e
    rejected=strcmp(e.identifier,id);
end
assert(rejected,'Invalid input or stage was accepted');
end

function restoreEnvironment(keys,values)
for k=1:numel(keys),setenv(keys{k},values{k});end
end
