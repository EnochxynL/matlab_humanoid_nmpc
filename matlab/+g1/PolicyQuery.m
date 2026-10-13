classdef PolicyQuery < matlab.System
    %POLICYQUERY Stateful OCS2 planner boundary; reference precedes query.
    % Initialization/cleanup belong to run_walking, not each Simulink block.
    methods (Access = protected)
        function policy = stepImpl(~, observation, reference)
            g1_ocp_mex('reference',reference);
            policy = g1_ocp_mex('query',observation);
        end
        function sz = getOutputSizeImpl(~)
            sz = [95 1];
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
