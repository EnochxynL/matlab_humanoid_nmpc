function g3a_start()
%G3A_START New simulation always creates a fresh session and clears actions.
global g3a_bridge g3a_options
if ~isempty(g3a_bridge)&&isvalid(g3a_bridge),delete(g3a_bridge);end
cfg=g3a_config();
w=get_param(cfg.modelName,'ModelWorkspace');cfg.transportEnabled=logical(getVariable(w,'g3a_use_ros'));
if ~isempty(g3a_options)
 names=fieldnames(g3a_options);
 for k=1:numel(names),cfg.(names{k})=g3a_options.(names{k});end
end
g3a_bridge=G1Ros2PlantBridge(cfg);
end
