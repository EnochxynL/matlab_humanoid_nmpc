function varargout = plannerMex(command,varargin)
%PLANNERMEX Coordinate OCS2 and independent inverse dynamics MEX instances.
% Simulink calls this wrapper; the native MEX never calls MATLAB callbacks.
persistent writer initialized inverseInitialized
if isempty(initialized), initialized = false; end
if isempty(inverseInitialized), inverseInitialized = false; end
if strcmp(command,'init')
    g1_ocp_mex(command,varargin{:});
    initialized = true;
    try
        config = varargin{1};
        g1_inverse_dynamics_mex('init',config);
        inverseInitialized = true;
        if ~isfield(config,'diagnosticsEnabled') || config.diagnosticsEnabled
            writer = g1.DiagnosticWriter(config.logDirectory);
        end
        collect();
    catch exception
        closeNative();
        rethrow(exception);
    end
elseif strcmp(command,'flush')
    if initialized, collect(); end
    if ~isempty(writer), writer.flush(); end
elseif strcmp(command,'close')
    cleanup = onCleanup(@closeNative);
    if initialized, collect(); end
    clear cleanup;
else
    try
        if strcmp(command,'inverse_dynamics')
            if nargout ~= 1 || numel(varargin) ~= 1
                error('G1:PlannerMex','inverse_dynamics requires policy95 and one output');
            end
            policy = varargin{1};
            validateattributes(policy,{'double'},{'real','finite','nonsparse','numel',95});
            torque = g1_inverse_dynamics_mex('compute',policy(1:58),policy(59:93));
            % 原生 OCS2 入口只核对策略身份、接收力矩并记录本 tick 的诊断。
            g1_ocp_mex('complete_action',policy,torque);
            varargout{1} = torque;
            collect();
        else
            [varargout{1:nargout}] = g1_ocp_mex(command,varargin{:});
        end
    catch exception
        % Preserve partial diagnostic evidence without hiding native failures.
        if initialized
            try
                collect();
            catch
                % Keep the original native error if evidence collection fails.
            end
        end
        rethrow(exception);
    end
end
    function collect()
        if isempty(writer), return; end
        batch = g1_ocp_mex('diagnostics');
        writer.append(batch);
    end
    function closeNative()
        inverseCleanup = onCleanup(@closeInverse);
        if initialized
            g1_ocp_mex('close'); initialized = false;
        end
        if ~isempty(writer), delete(writer); writer = []; end
        clear inverseCleanup;
    end
    function closeInverse()
        if inverseInitialized
            g1_inverse_dynamics_mex('close'); inverseInitialized = false;
        end
    end
end
