function cfg = g1_config()
%G1_CONFIG Central physical configuration; no MPC or workspace reset.
root = fileparts(fileparts(mfilename('fullpath')));
cfg = jsondecode(fileread(fullfile(root,'data','g1_source.json')));
if iscell(cfg.joints)
    % Optional URDF attributes can produce heterogeneous JSON records.
    for k = 1:numel(cfg.joints)
        if ~isfield(cfg.joints{k},'velocity'), cfg.joints{k}.velocity = NaN; end
    end
    cfg.joints = vertcat(cfg.joints{:});
end
cfg.projectRoot = root;
cfg.modelName = 'g1_29dof_plant';
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
end
