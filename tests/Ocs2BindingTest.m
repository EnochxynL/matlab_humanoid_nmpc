classdef Ocs2BindingTest < matlab.unittest.TestCase
    % 两种问题共享执行器，但各自与原 Controller 在相同冻结输入下比较。
    properties (TestParameter)
        formulation={'whole_body','centroidal'}
    end
    methods (TestClassSetup)
        function initializePaths(~)
            root=fileparts(fileparts(mfilename('fullpath')));
            addpath(fullfile(root,'matlab'),fullfile(root,'tools'),fullfile(root,'tests'));
            g1.initialize();
        end
    end
    methods (Test)
        function frozenReplay(testCase,formulation)
            result=binding_replay(formulation);
            testCase.verifyEqual(result.ticks,2401);
            testCase.verifyTrue(result.exact_equal);
            testCase.verifyEqual(result.max_absolute_error,0);
            testCase.verifyTrue(result.repeated_query_exact);
            testCase.verifyTrue(result.copy_counts_exact);
        end
        function lifecycleAndBoundary(testCase)
            result=binding_boundary_checks();
            testCase.verifyTrue(result.multiple_objects);
            testCase.verifyTrue(result.stale_handle_rejected);
            testCase.verifyTrue(result.wrong_dimension_rejected);
            testCase.verifyTrue(result.query_before_solve_rejected);
            testCase.verifyTrue(result.finite_check_rejected);
            testCase.verifyTrue(result.unlocked);
        end
    end
end
