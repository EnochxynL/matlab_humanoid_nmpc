function cleanup = useBuildCache()
%USEBUILDCACHE Scope generated Simulink files to this application's build tree.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
cacheRoot = fullfile(root,'build','matlab');
if ispc
    runtimeDirectory = getenv('G1_RUNTIME_DIRECTORY');
    if isempty(runtimeDirectory)
        runtimeDirectory = fullfile(root,'install','windows','lib');
    end
    metadataFile = fullfile(runtimeDirectory,'compiler-info.json');
    if isfile(metadataFile)
        metadata = jsondecode(fileread(metadataFile));
        cacheRoot = fullfile(cacheRoot,metadata.tag);
    end
end
config = Simulink.fileGenControl('getConfig');
cleanup = onCleanup(@()restore(config));
Simulink.fileGenControl('set', ...
    'CacheFolder',fullfile(cacheRoot,'simulink-cache'), ...
    'CodeGenFolder',fullfile(cacheRoot,'simulink-codegen'), ...
    'keepPreviousPath',true,'createDir',true);
end

function restore(config)
Simulink.fileGenControl('set','CacheFolder',config.CacheFolder, ...
    'CodeGenFolder',config.CodeGenFolder, ...
    'CodeGenFolderStructure',config.CodeGenFolderStructure,'keepPreviousPath',true);
end
