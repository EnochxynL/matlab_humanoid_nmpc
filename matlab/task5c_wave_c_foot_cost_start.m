function task5c_wave_c_foot_cost_start()
% Same accepted ROS plant bridge, new derived-model lifecycle only.
global g3a_bridge g3a_options task5a_runtime
if ~isempty(g3a_bridge)&&isvalid(g3a_bridge),delete(g3a_bridge);end
cfg=g3a_config();cfg.modelName='g1_wb_mpc_wave_c_foot_cost';cfg.transportEnabled=true;
if ~isempty(g3a_options)
 names=fieldnames(g3a_options);for k=1:numel(names),cfg.(names{k})=g3a_options.(names{k});end
end
task5b_config=task5b_load_config(task5a_runtime.folder);assignin('base','task5b_config',task5b_config);
g3a_bridge=G1Ros2PlantBridge(cfg);
assert(~isempty(task5a_runtime)&&isfolder(task5a_runtime.folder));
task5a_runtime.sequence=uint64(0);task5a_runtime.lastPublished=intmax('uint64');task5a_runtime.packets={};
end
