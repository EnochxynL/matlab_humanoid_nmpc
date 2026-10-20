classdef inverseDynamicsMexTest < matlab.unittest.TestCase
    % 两个 MEX 只交换 MATLAB 数组，各自持有自己的运行对象。
    properties
        Config
    end
    methods (TestMethodSetup)
        function configure(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            g1.initialize();
            g1.plannerMex('close');
            closeRaw();
            testCase.addTeardown(@closeRaw);
            testCase.addTeardown(@()g1.plannerMex('close'));
            testCase.Config = g1.plannerConfiguration(root,tempname,false);
        end
    end
    methods (Test)
        function independentModel(testCase)
            g1_inverse_dynamics_mex('init',testCase.Config);
            testCase.verifyTrue(mislocked('g1_inverse_dynamics_mex'));
            testCase.verifyFalse(mislocked('g1_ocp_mex'));
            first = compute(zeros(58,1),zeros(35,1));
            second = compute(zeros(58,1),zeros(35,1));
            testCase.verifySize(first,[23 1]);
            testCase.verifyTrue(all(isfinite(first)));
            testCase.verifyEqual(second,first,AbsTol=0);
            g1_inverse_dynamics_mex('close');
            g1_inverse_dynamics_mex('init',testCase.Config);
            testCase.verifyEqual(compute(zeros(58,1),zeros(35,1)),first,AbsTol=0);
        end
        function invalidInput(testCase)
            testCase.verifyError(@()compute(zeros(58,1),zeros(35,1)), ...
                'G1:InverseDynamicsMex');
            g1_inverse_dynamics_mex('init',testCase.Config);
            testCase.verifyError(@()compute(zeros(57,1),zeros(35,1)), ...
                'G1:InverseDynamicsMex');
            testCase.verifyError(@()compute(nan(58,1),zeros(35,1)), ...
                'G1:InverseDynamicsMex');
            testCase.verifyError(@()compute(sparse(58,1),zeros(35,1)), ...
                'G1:InverseDynamicsMex');
        end
        function failedModelInitLeavesNoLock(testCase)
            bad = testCase.Config;
            bad.urdfFile = fullfile(tempdir,'missing-g1-split-model.urdf');
            testCase.verifyError(@()g1_inverse_dynamics_mex('init',bad), ...
                'G1:InverseDynamicsMex');
            testCase.verifyFalse(mislocked('g1_inverse_dynamics_mex'));
            g1.plannerMex('init',testCase.Config);
            testCase.verifyTrue(mislocked('g1_ocp_mex'));
            testCase.verifyTrue(mislocked('g1_inverse_dynamics_mex'));
            g1.plannerMex('close');
            testCase.verifyFalse(mislocked('g1_ocp_mex'));
            testCase.verifyFalse(mislocked('g1_inverse_dynamics_mex'));
        end
        function existingIndependentModelSurvivesWrapperFailure(testCase)
            g1_inverse_dynamics_mex('init',testCase.Config);
            testCase.verifyError(@()g1.plannerMex('init',testCase.Config), ...
                'G1:InverseDynamicsMex');
            testCase.verifyFalse(mislocked('g1_ocp_mex'));
            testCase.verifyTrue(mislocked('g1_inverse_dynamics_mex'));
            testCase.verifySize(compute(zeros(58,1),zeros(35,1)),[23 1]);
        end
    end
end

function torque = compute(state,input)
torque = g1_inverse_dynamics_mex('compute',state,input);
end
function closeRaw()
g1_ocp_mex('close');
g1_inverse_dynamics_mex('close');
end
