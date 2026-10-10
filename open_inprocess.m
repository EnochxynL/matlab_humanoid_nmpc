function open_inprocess()
% Open the dataflow model without starting a controller or simulation.
root=fileparts(mfilename('fullpath'));cd(root);
addpath(fullfile(root,'matlab'));addpath(fullfile(root,'models'));addpath(fullfile(root,'build/controller'));
g1c_define_buses;task5a_define_buses;
open_system(fullfile(root,'models/g1_inprocess_walking.slx'));
fprintf('Ready: out=run_inprocess_walking starts the genuine 4.8 s run.\n');
end
