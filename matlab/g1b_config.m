function cfg = g1b_config()
%G1B_CONFIG Interface contract around the accepted, unchanged physical plant.
cfg = g1_config();
cfg.modelName = 'g1_jointdrive_plant';
cfg.modelFile = fullfile(cfg.projectRoot,'models',[cfg.modelName '.slx']);
cfg.actionTs = 0.002;
cfg.driveTs = 0.0005;
audit = jsondecode(fileread(fullfile(cfg.projectRoot,'data','g1_actuation_limits.json')));
assert(strcmp(audit.source_commit,cfg.source_commit));
assert(isequal(string({audit.joints.joint}),cfg.jointNames));
cfg.tauMax = [audit.joints.upper_Nm]';
cfg.actionFields = {'q_des','qd_des','kp','kd','tau_ff'};
cfg.driveFields = {'tau_ff','tau_p','tau_d','tau_raw','tau_applied', ...
    'tau_saturation_error','saturation_flag'};
cfg.stateFields = {'sim_time','base_position_world','base_orientation_wxyz', ...
    'base_linear_velocity_local','base_angular_velocity_local','q','qd'};
cfg.worldFields = {'pelvis_position_world','pelvis_R_world','pelvis_w_world','pelvis_v_world'};
end
