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
    if ~isempty(getenv('G1_RUNTIME_DIRECTORY')),runtimeDirectory=getenv('G1_RUNTIME_DIRECTORY');end
    pathParts = strsplit(getenv('PATH'),pathsep);
    if ~any(strcmpi(pathParts,runtimeDirectory))
        setenv('PATH',[runtimeDirectory pathsep getenv('PATH')]);
    end
end
metadataFile=fullfile(runtimeDirectory,'compiler-info.json');
cacheTag='default';
if isfile(metadataFile)
    metadata=jsondecode(fileread(metadataFile));
    cacheTag=metadata.tag;
    if ispc
        [status,compilerLocations]=system('where gcc');
        assert(status==0,'Start MATLAB with the UCRT64 compiler on PATH.');
        compilerLocations=splitlines(strtrim(compilerLocations));
        compiler=compilerLocations{1};
        [status,version]=system(['"' compiler '" -dumpfullversion']);
        assert(status==0 && strcmp(extractBefore(strtrim(version),'.'), ...
            extractBefore(metadata.version,'.')), ...
            'The compiler on PATH does not match the deployed controller.');
        [status,defines]=system(['"' compiler '" -dM -E -include _mingw.h -xc NUL']);
        assert(status==0 && contains(defines,'#define _UCRT'), ...
            'The deployed controller requires the UCRT64 compiler environment.');
        setenv('MW_MINGW64_LOC',fileparts(fileparts(compiler)));
    end
end
defaultCache=fullfile(root,'native','build','cppad_code_gen',cacheTag);
currentCache=getenv('G1_AD_CACHE_ROOT');
if isempty(currentCache) || strcmp(currentCache,getenv('G1_DEFAULT_AD_CACHE_ROOT')) || ...
        strcmpi(strrep(currentCache,'/',filesep),fullfile(root,'native','build','cppad_code_gen'))
    setenv('G1_AD_CACHE_ROOT',defaultCache);
    setenv('G1_DEFAULT_AD_CACHE_ROOT',defaultCache);
end
assert(isfile(fullfile(runtimeDirectory,['g1_ocp_mex.' mexext])), ...
    buildInstruction);
addpath(runtimeDirectory);
cfg = g1.loadConfiguration();
end
