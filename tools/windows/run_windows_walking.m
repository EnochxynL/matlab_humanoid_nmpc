function out = run_windows_walking()
%RUN_WINDOWS_WALKING Use Windows-only build caches without editing the model.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
previousFolder = pwd;
config = Simulink.fileGenControl('getConfig');
previousCache = config.CacheFolder;
previousCode = config.CodeGenFolder;
previousStructure = config.CodeGenFolderStructure;
cleanup = onCleanup(@()restore(previousFolder,previousCache,previousCode,previousStructure));
Simulink.fileGenControl('set', ...
    'CacheFolder',fullfile(root,'build','windows','simulink-cache'), ...
    'CodeGenFolder',fullfile(root,'build','windows','simulink-codegen'), ...
    'keepPreviousPath',true,'createDir',true);
cd(root);
out = run_walking();
end

function restore(folder,cache,code,structure)
Simulink.fileGenControl('set','CacheFolder',cache,'CodeGenFolder',code, ...
    'CodeGenFolderStructure',structure,'keepPreviousPath',true);
cd(folder);
end
