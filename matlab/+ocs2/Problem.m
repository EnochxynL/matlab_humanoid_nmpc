classdef Problem < handle
    %PROBLEM 实验性绑定：拥有一个原生问题及其可选 SQP/MRT 对象。
    % 调度、时间身份、初始目标和查询时间均由调用者明确提供。
    % 删除本对象会依次释放 MRT、求解器和问题，不能跨 MEX 共享指针。
    properties (SetAccess=private)
        Dimensions
    end
    properties (Access=private)
        NativeHandle
    end
    methods
        function obj=Problem(config)
            obj.NativeHandle=ocs2_binding_mex('create_problem',config);
            obj.Dimensions=ocs2_binding_mex('dimensions',obj.NativeHandle);
        end
        function delete(obj)
            if ~isempty(obj.NativeHandle)
                token=obj.NativeHandle;obj.NativeHandle=[];
                ocs2_binding_mex('destroy',token);
            end
        end
        function createSolver(obj)
            ocs2_binding_mex('create_solver',obj.NativeHandle);
        end
        function disableGaitExtension(obj)
            % humanoid 专用：由外部提供完整接触时间表。
            ocs2_binding_mex('disable_gait_extension',obj.NativeHandle);
        end
        function reset(obj,times,states,inputs)
            ocs2_binding_mex('reset',obj.NativeHandle,times,states,inputs);
        end
        function setTargetTrajectories(obj,times,states,inputs)
            ocs2_binding_mex('set_target',obj.NativeHandle,times,states,inputs);
        end
        function setModeSchedule(obj,eventTimes,modes)
            % 原样传递；GaitSchedule 所需的末尾标记由 MATLAB 调用者提供。
            ocs2_binding_mex('set_mode_schedule',obj.NativeHandle,eventTimes,modes);
        end
        function setCurrentObservation(obj,time,state,input,mode)
            ocs2_binding_mex('set_observation',obj.NativeHandle,time,state,input,mode);
        end
        function advanceMpc(obj)
            ocs2_binding_mex('advance_mpc',obj.NativeHandle);
        end
        function changed=updatePolicy(obj)
            changed=ocs2_binding_mex('update_policy',obj.NativeHandle);
        end
        function [state,input,mode]=evaluatePolicy(obj,time,measuredState)
            [state,input,mode]=ocs2_binding_mex('evaluate_policy',obj.NativeHandle,time,measuredState);
        end
        function stats=copyStats(obj)
            % 各项为 [显式搬运次数, 元素总数]；不包括 OCS2 内部和 MATLAB 分配。
            stats=ocs2_binding_mex('copy_stats',obj.NativeHandle);
        end
        function settings=modelSettings(obj)
            settings=ocs2_binding_mex('model_settings',obj.NativeHandle);
        end
        function state=convertObservation(obj,qv)
            state=ocs2_binding_mex('convert_observation',obj.NativeHandle,qv);
        end
        function enableDiagnostics(obj)
            ocs2_binding_mex('enable_diagnostics',obj.NativeHandle);
        end
        function batch=drainDiagnostics(obj)
            batch=ocs2_binding_mex('diagnostics',obj.NativeHandle);
        end
        function diagnosticTick(obj,tick)
            ocs2_binding_mex('diagnostic_tick',obj.NativeHandle,tick);
        end
        function trace(obj,event,time)
            ocs2_binding_mex('diagnostic_trace',obj.NativeHandle,event,time);
        end
        function diagnosticReference(obj,time,command)
            ocs2_binding_mex('diagnostic_reference',obj.NativeHandle,time,command);
        end
        function reference=referenceSnapshot(obj)
            reference=ocs2_binding_mex('reference_snapshot',obj.NativeHandle);
        end
    end
end
