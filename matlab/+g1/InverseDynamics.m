classdef InverseDynamics < matlab.System
    %INVERSEDYNAMICS Original numerical kernel behind a visible policy input.
    % MEX checks policy identity and rejects stale or altered packets.
    methods (Access = protected)
        function torque = stepImpl(~, policy)
            torque = g1_ocp_mex('inverse_dynamics',policy);
        end
        function sz = getOutputSizeImpl(~)
            sz = [23 1];
        end
        function dt = getOutputDataTypeImpl(~)
            dt = 'double';
        end
        function value = isOutputComplexImpl(~)
            value = false;
        end
        function value = isOutputFixedSizeImpl(~)
            value = true;
        end
    end
end
