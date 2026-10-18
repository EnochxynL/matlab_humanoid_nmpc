function result = validate_cfunction_trial
% 隔离验证块内 C 公式；不加载控制器或推进 Simscape。
[kernels,blockCleanup]=planner_block_functions();
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'build','standalone','lib'));
g = load(fullfile(root,'build','observation_hybrid','conversion_values.mat'));
open_system(fullfile(root,'models','experiments','g1_cfunction_trial.slx'));
n = size(g.raw,1);
in = Simulink.SimulationInput('g1_cfunction_trial');
in = in.setVariable('quaternionInput',[(0:n-1)'*.002,g.raw(:,5:8)]);
in = in.setModelParameter('StopTime',sprintf('%.17g',(n-1)*.002));
out = sim(in);
expected = zeros(n,3);
for k=1:n
    expected(k,:) = kernels.quaternionToEulerZyx(g.raw(k,5:8)')';
end
result.samples = n;
result.max_abs = max(abs(out.eulerOutput-expected),[],'all');
result.different_bits = nnz(typecast(out.eulerOutput(:),'uint64') ~= typecast(expected(:),'uint64'));
result.passed = result.different_bits == 0;
folder = fullfile(root,'build','cfunction_trial');
if ~isfolder(folder), mkdir(folder); end
save(fullfile(folder,'rerun.mat'),'result','out','expected');
disp(result);
assert(result.passed,'G1:CFunctionStrictGate','C Function does not match the accepted MEX bitwise.');
end
