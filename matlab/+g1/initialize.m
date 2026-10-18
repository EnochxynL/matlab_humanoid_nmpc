function cfg = initialize()
%INITIALIZE Add this project's runtime paths without resetting MATLAB state.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root,'matlab'),fullfile(root,'models'), ...
    fullfile(root,'tools'));
runtimeDirectory = fullfile(root,'build','standalone','lib');
buildInstruction = 'Build scripts/build_mex.sh first.';
if ispc
    runtimeDirectory = fullfile(root,'build','windows','core','lib');
    buildInstruction = 'Run scripts/build_windows.ps1 -Stage Core first.';
    if isempty(getenv('MW_MINGW64_LOC'))
        setenv('MW_MINGW64_LOC',fullfile(root,'build','windows','toolchains','mingw64'));
    end
    if isempty(getenv('G1_AD_CACHE_ROOT'))
        setenv('G1_AD_CACHE_ROOT',fullfile(root,'build','windows','ad-cache'));
    end
    pathParts = strsplit(getenv('PATH'),pathsep);
    if ~any(strcmpi(pathParts,runtimeDirectory))
        setenv('PATH',[runtimeDirectory pathsep getenv('PATH')]);
    end
end
assert(isfile(fullfile(runtimeDirectory,['g1_ocp_mex.' mexext])), ...
    buildInstruction);
addpath(runtimeDirectory);
cfg = g1.loadConfiguration();
end
