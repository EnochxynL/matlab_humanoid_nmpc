function result = validate_observation_conversion(evidenceName)
%VALIDATE_OBSERVATION_CONVERSION Compare actual Simulink blocks to C++ reference.
[kernels,blockCleanup]=planner_block_functions();
if nargin==0,evidenceName='observation_dataflow';end
root=fileparts(fileparts(mfilename('fullpath')));cd(root);cfg=g1.initialize();
folder=fullfile(root,'runtime','evidence',evidenceName);
if ~isfolder(folder),mkdir(folder);end
contract=jsondecode(fileread(fullfile(root,'build/observation_dataflow/conversion_contract.json')));
f=load(fullfile(root,'build/core_extraction/fixture.mat'));raw=f.raw;
% Independent bounded poses, velocities and joint excitations; no physical rollout.
extra=zeros(40,74);
for k=1:40
    yaw=-pi+2*pi*(k-1)/39;pitch=1.2*sin(k);roll=0.7*cos(2*k);
    cy=cos(yaw/2);sy=sin(yaw/2);cp=cos(pitch/2);sp=sin(pitch/2);cr=cos(roll/2);sr=sin(roll/2);
    q=[cr*cp*cy+sr*sp*sy;sr*cp*cy-cr*sp*sy;cr*sp*cy+sr*cp*sy;cr*cp*sy-sr*sp*cy];q=q/norm(q);
    extra(k,:)=raw(1,:);extra(k,1)=4.8+0.002*k;extra(k,2:4)=[0.1*k,-0.02*k,0.8];
    extra(k,5:8)=q;extra(k,9:14)=sin(k+(1:6));
    extra(k,15:43)=0.2*sin(k+(1:29));extra(k,44:72)=0.5*cos(k+(1:29));
end
raw=[raw;extra];t=(0:size(raw,1)-1)'*0.002;
ds=Simulink.SimulationData.Dataset;ds{1}=timeseries(raw,t);
in=Simulink.SimulationInput('g1_observation_check');in=in.setExternalInput(ds);
in=in.setModelParameter('StopTime',num2str(t(end),17),'SaveOutput','on','OutputSaveName','yout','SaveFormat','Dataset');
out=sim(in);actual=g1log.signalMatrix(out.yout{1}.Values);
golden=load(fullfile(root,'runtime','evidence','observation_hybrid','conversion_values.mat'));
assert(isequal(raw,golden.raw),'Frozen reference input changed');
expected=golden.expected;
assert(isequal(actual(:,[1:3,7:29,36:58]),expected(:,[1:3,7:29,36:58])),'Copied fields/joint mapping differs.');
error=abs(actual-expected);budget=contract.continuous_conversion.absolute_tolerance+contract.continuous_conversion.relative_tolerance*abs(expected);
assert(all(error<=budget,'all'),'Conversion exceeds frozen tolerance.');
assert(all(isfinite(actual),'all'));
% A wrong external joint mapping must be rejected by the runtime before solve.
bad=expected(1,:)';bad(7)=bad(7)+0.01;rejected=false;
try
    ignored=kernels.extractObservation(raw(1,:)',bad,double(cfg.mpcIndices(:)')); %#ok<NASGU>
catch exception
    rejected=strcmp(exception.identifier,'G1:ObservationExtraction');
end
assert(rejected,'Incorrect joint mapping was accepted.');
result=struct('passed',true,'samples',size(raw,1),'saved_observations',size(f.raw,1),'independent_cases',40, ...
    'maximum_absolute_error',max(error,[],'all'),'copied_fields_exact',true,'mapping_guard_passed',true, ...
    'joint_indices',cfg.mpcIndices(:)','physical_simulation',false);
g1log.writeJson(fullfile(folder,'conversion_acceptance.json'),result);
save(fullfile(folder,'conversion_values.mat'),'raw','actual','expected','result');disp(result);
end
function restoreEnvironment(keys,values)
for k=1:numel(keys),setenv(keys{k},values{k});end
end
