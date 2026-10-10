function open_project()
%OPEN_PROJECT Open the walking model; do not start a controller or simulation.
root = fileparts(mfilename('fullpath'));
cd(root); addpath(fullfile(root,'matlab'));
cfg = g1.initialize();
open_system(cfg.modelFile);
fprintf('Ready: run_walking starts the real 4.8 s Simscape experiment.\n');
end
