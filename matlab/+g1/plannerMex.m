function varargout = plannerMex(command,varargin)
%PLANNERMEX Coordinate OCS2 and independent inverse dynamics MEX instances.
% Simulink calls this wrapper; the native MEX never calls MATLAB callbacks.
persistent writer initialized inverseInitialized backend
persistent queryOnly centroidal
persistent currentToken rawTime lastPolicy lastTorque
persistent lastExecutionInput
if isempty(initialized), initialized = false; end
if isempty(inverseInitialized), inverseInitialized = false; end
if isempty(queryOnly), queryOnly = false; end
if isempty(centroidal), centroidal = false; end
if strcmp(command,'init')
    if initialized,error('G1:PlannerMex','Close existing planner before initialization');end
    try
        config = varargin{1};
        if ~isfield(config,'formulation'),config.formulation='whole_body';end
        backend=g1.PlannerBackend(config);
        initialized = true;
        centroidal = isfield(config,'formulation') && strcmp(config.formulation,'centroidal');
        queryOnly = centroidal && (~isfield(config,'executionEnabled') || ~config.executionEnabled);
        if ~queryOnly
            g1_inverse_dynamics_mex('init',config);
            inverseInitialized = true;
        end
        currentToken = []; rawTime = []; lastPolicy = []; lastTorque = [];
        lastExecutionInput = [];
        if ~isfield(config,'diagnosticsEnabled') || config.diagnosticsEnabled
            writer = g1.DiagnosticWriter(config.logDirectory);
        end
        collect();
    catch exception
        closeNative();
        plannerError(exception);
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
        if strcmp(command,'centroidal_inverse_dynamics')
            if ~centroidal || queryOnly || nargout~=1 || numel(varargin)~=3
                error('G1:PlannerMex','Centroidal execution requires policy72, measured state58 and joint acceleration23');
            end
            policy=varargin{1}; measured=varargin{2}; acceleration=varargin{3};
            validateVector(policy,72); validateVector(measured,58); validateVector(acceleration,23);
            policy=policy(:);
            if isempty(lastPolicy) || ~isequal(policy(:),lastPolicy)
                error('G1:PlannerMex','Inverse dynamics requires the current queried policy');
            end
            executionInput=[measured(:);acceleration(:)];
            if ~isempty(lastTorque) && ~isequal(lastExecutionInput,executionInput)
                error('G1:PlannerMex','Repeated centroidal execution changed its measured state or acceleration');
            end
            if isempty(lastTorque)
                lastTorque=g1_inverse_dynamics_mex('compute',measured, ...
                    [policy(36:47);acceleration(:)]);
                lastExecutionInput=executionInput;
                if ~isempty(writer)
                    writer.sample(currentToken(1),rawTime,currentToken(2), ...
                        [lastPolicy(1:70);lastTorque;lastPolicy(71:72)],'centroidal_samples');
                end
                collect();
            end
            varargout{1}=lastTorque;
        elseif strcmp(command,'inverse_dynamics')
            if centroidal
                error('G1:PlannerMex','Use the centroidal execution adapter for a centroidal policy');
            end
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
            nativeArguments = varargin;
            if strcmp(command,'install_observation')
                if numel(varargin) ~= 2
                    error('G1:PlannerMex','install_observation requires state and time identity');
                end
                nativeArguments{2} = validateObservationTime(varargin{2});
            end
            if strcmp(command,'install_observation') && numel(varargin) == 2 && ...
                    isnumeric(varargin{2}) && numel(varargin{2}) == 3 && ...
                    ~isempty(currentToken) && varargin{2}(1) ~= currentToken(1) && ...
                    ((queryOnly && isempty(lastPolicy)) || (~queryOnly && isempty(lastTorque)))
                error('G1:PlannerMex','Complete current action before advancing tick');
            end
            if ~initialized,error('G1:PlannerMex','Initialize planner first');end
            [varargout{1:nargout}] = backend.execute(command,nativeArguments{:});
            if strcmp(command,'install_observation')
                token = varargout{1};
                if isempty(currentToken) || token(1) ~= currentToken(1)
                    currentToken = token; rawTime = varargin{2}(3);
                    lastPolicy = []; lastTorque = [];
                    lastExecutionInput = [];
                    if ~isempty(writer)
                        writer.timeIdentity(token(1),rawTime,token(2));
                    end
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
        plannerError(exception);
    end
end

    function collect()
        if isempty(writer), return; end
        batch = backend.execute('diagnostics');
        writer.append(batch);
    end
    function closeNative()
        inverseCleanup = onCleanup(@closeInverse);
        if ~isempty(backend),delete(backend);backend=[];end
        initialized = false;
        if ~isempty(writer), delete(writer); writer = []; end
        currentToken = []; rawTime = []; lastPolicy = []; lastTorque = [];
        lastExecutionInput = [];
        queryOnly = false;
        centroidal = false;
        clear inverseCleanup;
    end
    function closeInverse()
        if inverseInitialized
            g1_inverse_dynamics_mex('close'); inverseInitialized = false;
        end
    end
end

function plannerError(exception)
if strcmp(exception.identifier,'G1:PlannerMex'),rethrow(exception);end
wrapped=MException('G1:PlannerMex','%s',exception.message);
throwAsCaller(addCause(wrapped,exception));
end

function validateVector(value,count)
if ~isa(value,'double') || ~isreal(value) || issparse(value) || ...
        numel(value)~=count || ~all(isfinite(value(:)))
    error('G1:PlannerMex','Invalid centroidal execution argument');
end
end

function identity = validateObservationTime(extracted)
% MATLAB 维护 2 ms 物理时钟；原生接口只接收 tick 和规范时间。
if ~isa(extracted,'double') || ~isreal(extracted) || issparse(extracted) || ...
        numel(extracted) ~= 3 || ~all(isfinite(extracted(:)))
    error('G1:PlannerMex','Expected finite double [tick, canonical time, raw time]');
end
tick = extracted(1);
if tick < 0 || tick > flintmax || tick ~= floor(tick)
    error('G1:PlannerMex','Invalid extracted tick');
end
canonical = tick * 0.002;
if abs(extracted(3)-canonical) > 1e-12 || extracted(2) ~= canonical
    error('G1:PlannerMex','Extracted time does not match the physical tick');
end
identity = [tick;canonical];
end
