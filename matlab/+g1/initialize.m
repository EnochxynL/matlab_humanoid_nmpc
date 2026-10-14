function cfg = initialize()
%INITIALIZE Add this project's runtime paths without resetting MATLAB state.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root,'matlab'),fullfile(root,'models'), ...
    fullfile(root,'tools'));
runtimeDirectory = fullfile(root,'build','standalone','lib');
assert(isfile(fullfile(runtimeDirectory,['g1_ocp_mex.' mexext])), ...
    'Build scripts/build_mex.sh first.');
assert(isfile(fullfile(runtimeDirectory,['g1_observation_mex.' mexext])), ...
    'Build scripts/build_mex.sh for observation kernels.');
assert(isfile(fullfile(runtimeDirectory,['g1_observation_extract_mex.' mexext])), ...
    'Build scripts/build_mex.sh for observation extraction.');
addpath(runtimeDirectory);
cfg = g1.loadConfiguration();
end
