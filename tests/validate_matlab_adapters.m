function result = validate_matlab_adapters()
%VALIDATE_MATLAB_ADAPTERS Check native conversions, mappings and packet guards.
[kernels,blockCleanup]=planner_block_functions();
root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'matlab'),fullfile(root,'tools'));
cfg=g1.loadConfiguration();
golden=load(fullfile(root,'runtime','evidence','observation_hybrid','conversion_values.mat'));
maximum=0; different=0;
for j=1:size(golden.raw,1)
    raw=golden.raw(j,:)';
    euler=kernels.quaternionToEulerZyx(raw(5:8));
    world=kernels.rotateLinearVelocity(raw(5:8),raw(9:11));
    rates=kernels.localAngularVelocityToEulerRates(euler,raw(12:14));
    actual=[euler;world;rates]; expected=golden.expected(j,[4:6,30:35])';
    assert(all(isfinite(actual)));
    maximum=max(maximum,max(abs(actual-expected)));
    different=different+nnz(typecast(actual,'uint64')~=typecast(expected,'uint64'));
    % Gross conversion mistakes must still fail; last-bit differences are reported.
    assert(all(abs(actual-expected)<=1e-10*(1+abs(expected))));
    if j<=2401
        identity=kernels.extractObservation(raw,golden.expected(j,:)',double(cfg.mpcIndices(:)));
        assert(isequal(identity,[j-1;(j-1)*.002;raw(1)]));
    end
end
raw=golden.raw(1,:)'; state=golden.expected(1,:)'; indices=double(cfg.mpcIndices(:));
bad=raw; bad(4)=.2; reject(@()kernels.extractObservation(bad,state,indices));
bad=raw; bad(1)=.001; reject(@()kernels.extractObservation(bad,state,indices));
bad=raw; bad(5:8)=0; reject(@()kernels.extractObservation(bad,state,indices));
bad=raw; bad(9)=NaN; reject(@()kernels.extractObservation(bad,state,indices));
bad=state; bad(7)=bad(7)+1; reject(@()kernels.extractObservation(raw,bad,indices));
bad=indices; bad(2)=bad(1); reject(@()kernels.extractObservation(raw,state,bad));
reject(@()kernels.quaternionToEulerZyx(single(raw(5:8))));
reject(@()kernels.quaternionToEulerZyx([1;0;1;0]));
reject(@()kernels.rotateLinearVelocity(raw(5:8),[NaN;0;0]));
reject(@()kernels.localAngularVelocityToEulerRates([0;0;0],[0;0]));
fixture=load(fullfile(root,'runtime','evidence','core_extraction','fixture.mat'));
for j=1:size(fixture.ref,1)
    reference=fixture.ref(j,:)'; actual=kernels.extractReference(reference);
    assert(isequal(typecast(actual,'uint64'),typecast(reference,'uint64')));
end
reference=fixture.ref(find(fixture.ref(:,2)==1,1),:)';
assert(~isempty(reference));
bad=reference;bad(2)=2;reject(@()kernels.extractReference(bad));
bad=reference;bad(1)=.5;reject(@()kernels.extractReference(bad));
bad=reference;bad(3)=2;reject(@()kernels.extractReference(bad));
bad=reference;bad(286)=65;reject(@()kernels.extractReference(bad));
bad=reference;bad(351)=4;reject(@()kernels.extractReference(bad));
bad=reference;bad(416)=bad(287);reject(@()kernels.extractReference(bad));
bad=reference;bad(417)=.06;reject(@()kernels.extractReference(bad));
bad=reference;bad(10)=Inf;reject(@()kernels.extractReference(bad));
unpublished=zeros(428,1);unpublished(3)=-9;unpublished(20)=-0.0;
assert(isequal(typecast(kernels.extractReference(unpublished),'uint64'),typecast(unpublished,'uint64')));
result=struct('passed',true,'conversion_cases',size(golden.raw,1), ...
    'observation_ticks',2401,'reference_cases',size(fixture.ref,1), ...
    'negative_cases',18,'max_absolute_error',maximum,'different_double_bits',different, ...
    'reference_bits_exact',true,'physical_simulation',false);
folder=fullfile(root,'runtime','evidence','inline_matlab_migration');
if ~isfolder(folder),mkdir(folder);end
g1log.writeJson(fullfile(folder,'adapters.json'),result);
disp(result);
end

function reject(callback)
rejected=false;
try
    ignored=callback(); %#ok<NASGU>
catch
    rejected=true;
end
assert(rejected,'Invalid packet was accepted.');
end
