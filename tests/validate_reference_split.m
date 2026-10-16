function result=validate_reference_split(evidenceName)
if nargin==0,evidenceName='reference_split';end
%VALIDATE_REFERENCE_SPLIT Stateless reference contract, rejection cases and full policy replay.
root=fileparts(fileparts(mfilename('fullpath')));cd(root);g1.initialize();
folder=fullfile(root,'build',evidenceName);
f=load(fullfile(root,'build','core_extraction','fixture.mat'));
% No planner init: the independent MEX must operate on its own.
for j=1:size(f.ref,1)
    actual=g1_reference_extract_mex(f.ref(j,:)');
    assert(isequal(typecast(actual(:),'uint64'),typecast(f.ref(j,:)','uint64')),'Reference bits changed');
end
r=f.ref(find(f.ref(:,2)~=0,1),:)';
b=r;b(3)=2;reject(b);
b=r;b(286)=65;reject(b);
b=r;b(1)=-1;reject(b);
b=r;b(1)=.5;reject(b);
b=r;b(351)=4;reject(b);
b=r;b(417)=b(417)+.1;reject(b);
b=r;b(416)=b(287);reject(b);
b=r;b(3)=3;b(4:6)=[0;0;1];reject(b);
b=r;b(10)=NaN;reject(b);
b=r;b(2)=2;reject(b);
b=zeros(428,1);assert(isequal(g1_reference_extract_mex(b),b),'Unpublished reference changed');
replay=validate_planner_dataflow(evidenceName);
result=struct('passed',true,'reference_packets_exact',size(f.ref,1), ...
    'published_packets',nnz(f.ref(:,2)),'negative_cases',10,'replay',replay, ...
    'physical_simulation',false);
g1log.writeJson(fullfile(folder,'acceptance.json'),result);disp(result);
end
function reject(reference)
rejected=false;
try
    ignored=g1_reference_extract_mex(reference); %#ok<NASGU>
catch e
    rejected=strcmp(e.identifier,'G1:ReferenceExtractionMex');
end
assert(rejected,'Invalid reference accepted');
end
