function open_project()
%OPEN_PROJECT Open the walking model; do not start a controller or simulation.
root = fileparts(mfilename('fullpath'));
cd(root); addpath(fullfile(root,'matlab'));
cfg = g1.initialize();
open_system(cfg.modelFile);
fprintf('Model opened; MEX is not initialized yet.\n');
fprintf('Start from the MATLAB command window: out = run_walking;\n');
fprintf('Do not click Simulink Run directly: it bypasses controller initialization.\n');
end
