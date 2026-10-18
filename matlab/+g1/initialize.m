function cfg = initialize()
%INITIALIZE Add this project's runtime paths without resetting MATLAB state.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root,'matlab'),fullfile(root,'models'), ...
    fullfile(root,'tools'));
runtimeDirectory = fullfile(root,'install','linux','lib');
buildInstruction = 'Build native/scripts/build_mex.sh first.';
if ispc
    runtimeDirectory = fullfile(root,'install','windows','lib');
    buildInstruction = 'Run native/scripts/build_windows.ps1 -Stage Core first.';
    if isempty(getenv('MW_MINGW64_LOC')) || ...
            strcmpi(strrep(getenv('MW_MINGW64_LOC'),'/',filesep),fullfile(root,'build','windows','toolchains','mingw64'))
        setenv('MW_MINGW64_LOC',fullfile(root,'native','build','windows','toolchains','mingw64'));
    end
    pathParts = strsplit(getenv('PATH'),pathsep);
    if ~any(strcmpi(pathParts,runtimeDirectory))
        setenv('PATH',[runtimeDirectory pathsep getenv('PATH')]);
    end
end
if isempty(getenv('G1_AD_CACHE_ROOT')) || ...
        strcmpi(strrep(getenv('G1_AD_CACHE_ROOT'),'/',filesep),fullfile(root,'build','windows','ad-cache'))
    setenv('G1_AD_CACHE_ROOT',fullfile(root,'native','build','cppad_code_gen'));
end
assert(isfile(fullfile(runtimeDirectory,['g1_ocp_mex.' mexext])), ...
    buildInstruction);
addpath(runtimeDirectory);
cfg = g1.loadConfiguration();
end
