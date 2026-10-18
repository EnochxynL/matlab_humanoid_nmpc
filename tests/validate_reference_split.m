function result=validate_reference_split(evidenceName)
[kernels,blockCleanup]=planner_block_functions();
if nargin==0,evidenceName='reference_split';end
%VALIDATE_REFERENCE_SPLIT Stateless reference contract, rejection cases and full policy replay.
root=fileparts(fileparts(mfilename('fullpath')));cd(root);g1.initialize();
folder=fullfile(root,'runtime','evidence',evidenceName);
f=load(fullfile(root,'runtime','evidence','core_extraction','fixture.mat'));
% No planner init: the native function must operate on its own.
for j=1:size(f.ref,1)
    actual=kernels.extractReference(f.ref(j,:)');
    assert(isequal(typecast(actual(:),'uint64'),typecast(f.ref(j,:)','uint64')),'Reference bits changed');
end
r=f.ref(find(f.ref(:,2)~=0,1),:)';
b=r;b(3)=2;reject(b,kernels);
b=r;b(286)=65;reject(b,kernels);
b=r;b(1)=-1;reject(b,kernels);
b=r;b(1)=.5;reject(b,kernels);
b=r;b(351)=4;reject(b,kernels);
b=r;b(417)=b(417)+.1;reject(b,kernels);
b=r;b(416)=b(287);reject(b,kernels);
b=r;b(3)=3;b(4:6)=[0;0;1];reject(b,kernels);
b=r;b(10)=NaN;reject(b,kernels);
b=r;b(2)=2;reject(b,kernels);
b=zeros(428,1);assert(isequal(kernels.extractReference(b),b),'Unpublished reference changed');
replay=validate_planner_dataflow(evidenceName,true);
result=struct('passed',true,'reference_packets_exact',size(f.ref,1), ...
    'published_packets',nnz(f.ref(:,2)),'negative_cases',10,'replay',replay, ...
    'physical_simulation',false);
g1log.writeJson(fullfile(folder,'acceptance.json'),result);disp(result);
end
function reject(reference,kernels)
rejected=false;
try
    ignored=kernels.extractReference(reference); %#ok<NASGU>
catch e
    rejected=strcmp(e.identifier,'G1:ReferenceExtraction');
end
assert(rejected,'Invalid reference accepted');
end
