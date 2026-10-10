function out=run_walking()
% Accepted OCS2/Pinocchio/CppAD controller, genuine Simscape walking or standing.
root=setup_project();session=jsondecode(fileread(fullfile(root,'runtime/current_session.json')));
folder=session.folder;assert(isfolder(folder)&&startsWith(folder,fullfile(root,'runtime')));
assert(isfile(fullfile(folder,'loaded_configuration.json')),'Wait for controller.py READY before running MATLAB.');
cfg=g3a_config();cfg.modelName='g1_wb_mpc_wave_c_foot_cost';cfg.modelFile=fullfile(root,'models',[cfg.modelName '.slx']);
if bdIsLoaded(cfg.modelName)
 assert(strcmp(get_param(cfg.modelName,'FileName'),cfg.modelFile),'The same model name is open from another project. Close that model or use a fresh MATLAB session.');
end
global g3a_options g3a_bridge task5a_runtime
g3a_options=struct('replyTimeout',.25,'initialTimeout',3,'transportEnabled',true);
task5a_runtime=struct('folder',folder,'sequence',uint64(0),'lastPublished',intmax('uint64'),'packets',{{}});
data=g1b_synthetic_action(cfg,'zero',session.duration);in=g1b_sim_input(cfg,data,session.duration);
in=in.setVariable('g3a_use_ros',1,'Workspace',cfg.modelName);in=in.setVariable('task5a_standing',double(session.standing));
in=in.setModelParameter('MaxStep','0.0005','SimMechanicsOpenEditorOnUpdate','on','CaptureErrors','on');
open_system(cfg.modelFile);
if exist('layout.LayoutStateManager','class')
 hold=layout.LayoutStateManager.instance().getScopedLayoutListenerDisabler(cfg.modelName); %#ok<NASGU>
end
out=sim(in);clear hold;
% Save the complete real result even if the simulation reports an error.
save(fullfile(folder,'simulation_output.mat'),'out','session','-v7.3');
if ~isempty(out.ErrorMessage),error('Walking:SimulationFailed','%s',out.ErrorMessage);end
logs=g3a_log_dataset(out);events=g3a_bridge.Events;counts=g3a_bridge.Counts;packets=task5a_runtime.packets;
save(fullfile(folder,'plant_and_reference.mat'),'logs','events','counts','packets','-v7.3');
assert(~any(counts),'Walking:Protocol','Bridge protocol failure; complete evidence saved.');
fprintf('Completed %.3f s. Mechanics Explorer remains available; output: %s\n',session.duration,folder);
end
