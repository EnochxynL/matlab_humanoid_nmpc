function result = run_windows_planner_replay(evidenceName)
%RUN_WINDOWS_PLANNER_REPLAY Keep generated simulation files in Windows caches.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
if nargin==0,evidenceName='windows/planner_dataflow';end
previousFolder = pwd;
config = Simulink.fileGenControl('getConfig');
cleanup = onCleanup(@()restore(previousFolder,config));
Simulink.fileGenControl('set', ...
    'CacheFolder',fullfile(root,'build','windows','simulink-cache'), ...
    'CodeGenFolder',fullfile(root,'build','windows','simulink-codegen'), ...
    'keepPreviousPath',true,'createDir',true);
addpath(fullfile(root,'matlab'),fullfile(root,'tests'));
result = validate_planner_dataflow(evidenceName,true);
end

function restore(folder,config)
Simulink.fileGenControl('set','CacheFolder',config.CacheFolder, ...
    'CodeGenFolder',config.CodeGenFolder, ...
    'CodeGenFolderStructure',config.CodeGenFolderStructure,'keepPreviousPath',true);
cd(folder);
end
