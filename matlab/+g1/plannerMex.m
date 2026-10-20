function varargout = plannerMex(command,varargin)
%PLANNERMEX Run native operations and collect diagnostic batches in MATLAB.
% Simulink calls this wrapper; the native MEX never calls MATLAB callbacks.
persistent writer initialized
if isempty(initialized), initialized = false; end
if strcmp(command,'init')
    g1_ocp_mex(command,varargin{:});
    initialized = true;
    try
        config = varargin{1};
        if ~isfield(config,'diagnosticsEnabled') || config.diagnosticsEnabled
            writer = g1.DiagnosticWriter(config.logDirectory);
        end
        collect();
    catch exception
        g1_ocp_mex('close'); initialized = false;
        if ~isempty(writer), delete(writer); writer = []; end
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
        [varargout{1:nargout}] = g1_ocp_mex(command,varargin{:});
        if strcmp(command,'inverse_dynamics'), collect(); end
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
        g1_ocp_mex('close'); initialized = false;
        if ~isempty(writer), delete(writer); writer = []; end
    end
end
