function cfg = loadConfiguration()
%LOADCONFIGURATION Read the accepted robot, drive and contact contracts.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
cfg = jsondecode(fileread(fullfile(root,'data','g1_source.json')));
if iscell(cfg.joints)
    % Optional URDF attributes can produce heterogeneous JSON records.
    for k = 1:numel(cfg.joints)
        if ~isfield(cfg.joints{k},'velocity'), cfg.joints{k}.velocity = NaN; end
    end
    cfg.joints = vertcat(cfg.joints{:});
end
cfg.projectRoot = root;
cfg.modelName = 'g1_walking';
cfg.modelFile = fullfile(root,'models',[cfg.modelName '.slx']);
cfg.jointNames = string({cfg.joints.name});
cfg.jointIndex = containers.Map(cellstr(cfg.jointNames),num2cell(1:29));
cfg.mpcIndices = cellfun(@(n) cfg.jointIndex(n),cfg.mpc_joint_names);
cfg.wristIndices = cellfun(@(n) cfg.jointIndex(n),cfg.non_mpc_joint_names);
cfg.initialQ = cfg.nominal_q(:);
cfg.initialQd = zeros(29,1);
cfg.initialPosition = cfg.initial_position(:);
cfg.initialRpy = cfg.initial_rpy(:);
cfg.gravity = [0 0 -9.81];
% Soft stops are simulator-specific; these are explicit SI starting values,
% not an asserted equivalence to MuJoCo's solref/solimp limit constraint.
cfg.stopStiffness = 1e4;       % N*m/rad
cfg.stopDamping = 100;        % N*m/(rad/s)
cfg.stopTransition = 1e-3;    % rad
cfg.maxStep = 5e-4;           % basic no-contact tests; not convergence claim
cfg.diagnosticLinks = [{cfg.root_frame}; cfg.foot_links; cfg.hand_links];
cfg.diagnosticNames = [{'pelvis'}; cfg.foot_frames; cfg.hand_links];

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

cfg.geometry=jsondecode(fileread(fullfile(cfg.projectRoot,'data','g1_contact_geometry.json')));
cfg.contactPoints=cfg.geometry.feet(1).point_cloud_contact_m;
cfg.groundHeight=0;
cfg.normalStiffness=1e5; cfg.normalDamping=1e3; cfg.transitionWidth=1e-3;
cfg.muStatic=1.5; cfg.muDynamic=1.2; cfg.criticalVelocity=1e-3;
cfg.contactThreshold=1; cfg.copThreshold=5;
cfg.fixtureMass=10; cfg.fixtureHeight=0.02;
cfg.footFields={'in_contact','normal_force','friction_force_world', ...
    'total_force_world','total_torque_world','penetration','wrench_local', ...
    'cop_local','cop_valid','point_normal_force','point_penetration'};
cfg.footDims=[1 1 3 3 3 1 6 2 1 4 4];
end
