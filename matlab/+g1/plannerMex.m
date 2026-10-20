function varargout = plannerMex(command,varargin)
%PLANNERMEX Coordinate OCS2 and independent inverse dynamics MEX instances.
% Simulink calls this wrapper; the native MEX never calls MATLAB callbacks.
persistent writer initialized inverseInitialized
persistent currentToken rawTime lastPolicy lastTorque
if isempty(initialized), initialized = false; end
if isempty(inverseInitialized), inverseInitialized = false; end
if strcmp(command,'init')
    g1_ocp_mex(command,varargin{:});
    initialized = true;
    try
        config = varargin{1};
        g1_inverse_dynamics_mex('init',config);
        inverseInitialized = true;
        currentToken = []; rawTime = []; lastPolicy = []; lastTorque = [];
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
            if ~isa(policy,'double') || ~isreal(policy) || issparse(policy) || ...
                    numel(policy) ~= 95 || ~all(isfinite(policy(:))) || ...
                    isempty(lastPolicy) || ~isequal(policy(:),lastPolicy)
                error('G1:PlannerMex','Inverse dynamics requires the current queried policy');
            end
            if ~isempty(lastTorque)
                varargout{1} = lastTorque;
                return;
            end
            torque = g1_inverse_dynamics_mex('compute',policy(1:58),policy(59:93));
            lastTorque = torque;
            if ~isempty(writer)
                writer.sample(currentToken(1),rawTime,currentToken(2), ...
                    [lastPolicy(1:93); torque; lastPolicy(94:95)]);
            end
            varargout{1} = torque;
            collect();
        else
            if strcmp(command,'install_observation') && numel(varargin) == 2 && ...
                    isnumeric(varargin{2}) && numel(varargin{2}) == 3 && ...
                    ~isempty(currentToken) && varargin{2}(1) ~= currentToken(1) && ...
                    isempty(lastTorque)
                error('G1:PlannerMex','Complete current action before advancing tick');
            end
            [varargout{1:nargout}] = g1_ocp_mex(command,varargin{:});
            if strcmp(command,'install_observation')
                token = varargout{1};
                if isempty(currentToken) || token(1) ~= currentToken(1)
                    currentToken = token; rawTime = varargin{2}(3);
                    lastPolicy = []; lastTorque = [];
                end
            elseif strcmp(command,'query_policy')
                lastPolicy = varargout{1}(:);
            end
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
        currentToken = []; rawTime = []; lastPolicy = []; lastTorque = [];
        clear inverseCleanup;
    end
    function closeInverse()
        if inverseInitialized
            g1_inverse_dynamics_mex('close'); inverseInitialized = false;
        end
    end
end
