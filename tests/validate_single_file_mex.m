function result = validate_single_file_mex()
%VALIDATE_SINGLE_FILE_MEX Frozen conversion values and full planner replay.
root=fileparts(fileparts(mfilename('fullpath')));cd(root);cfg=g1.initialize();
folder=fullfile(root,'build','single_file_mex');
golden=load(fullfile(root,'build','observation_hybrid','conversion_values.mat'));
actual=zeros(size(golden.raw,1),9);
% Explicitly unload the old stateless adapters before testing the new files.
clear g1_observation_mex g1_observation_extract_mex g1_reference_extract_mex;
for j=1:size(golden.raw,1)
    raw=golden.raw(j,:)';
    euler=g1_observation_mex('quaternion_to_euler',raw(5:8));
    world=g1_observation_mex('rotate_linear_velocity',raw(5:8),raw(9:11));
    rates=g1_observation_mex('angular_velocity_to_euler_rates',euler,raw(12:14));
    actual(j,:)=[euler;world;rates]';
end
expected=golden.expected(:,[4:6,30:35]);
assert(isequal(typecast(actual(:),'uint64'),typecast(expected(:),'uint64')), ...
    'Conversion bits changed after merging translation units');
indices=double(cfg.mpcIndices(:)');
for j=1:2401
    identity=g1_observation_extract_mex(golden.raw(j,:)',golden.expected(j,:)',indices);
    assert(isequal(identity,[double(j-1);double(j-1)*.002;golden.raw(j,1)]));
end
% Reuse the bounded rejection tests and one complete reference/policy replay.
raw=golden.raw(1,:)';state=golden.expected(1,:)';
bad=raw;bad(4)=.2;reject(@()g1_observation_extract_mex(bad,state,indices),'G1:ObservationExtractionMex');
bad=raw;bad(1)=.001;reject(@()g1_observation_extract_mex(bad,state,indices),'G1:ObservationExtractionMex');
bad=raw;bad(5:8)=0;reject(@()g1_observation_extract_mex(bad,state,indices),'G1:ObservationExtractionMex');
bad=raw;bad(9)=NaN;reject(@()g1_observation_extract_mex(bad,state,indices),'G1:ObservationExtractionMex');
bad=state;bad(7)=bad(7)+1;reject(@()g1_observation_extract_mex(raw,bad,indices),'G1:ObservationExtractionMex');
bad=indices;bad(2)=bad(1);reject(@()g1_observation_extract_mex(raw,state,bad),'G1:ObservationExtractionMex');
reference=validate_reference_split('single_file_mex');
result=struct('passed',true,'conversion_cases_exact',size(actual,1), ...
    'observation_ticks_exact',2401,'observation_negative_cases',6, ...
    'reference',reference,'physical_simulation',false);
g1log.writeJson(fullfile(folder,'acceptance.json'),result);disp(result);
end
function reject(callback,id)
rejected=false;
try
    ignored=callback(); %#ok<NASGU>
catch e
    rejected=strcmp(e.identifier,id);
end
assert(rejected,'Invalid input accepted');
end
