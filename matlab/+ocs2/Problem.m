classdef Problem < handle
    %PROBLEM 原生对象绑定：拥有一个原生问题及其可选 SQP/MRT 对象。
    % 调度、时间身份、初始目标和查询时间均由调用者明确提供。
    % 删除本对象会依次释放 MRT、求解器和问题，不能跨 MEX 共享指针。
    properties (SetAccess=private)
        Dimensions
    end
    properties (Access=private)
        NativeHandle
    end
    methods
        % 绑定辅助：对象生命周期、组装与模型转换
        function obj=Problem(config)
            obj.NativeHandle=ocs2_binding_mex('createProblem',config);
            obj.Dimensions=ocs2_binding_mex('dimensions',obj.NativeHandle);
        end
        function delete(obj)
            if ~isempty(obj.NativeHandle)
                token=obj.NativeHandle;obj.NativeHandle=[];
                ocs2_binding_mex('destroyProblem',token);
            end
        end
        function createSolver(obj)
            ocs2_binding_mex('createSolver',obj.NativeHandle);
        end
        function settings=modelSettings(obj)
            settings=ocs2_binding_mex('modelSettings',obj.NativeHandle);
        end
        function state=convertObservation(obj,qv)
            state=ocs2_binding_mex('convertObservation',obj.NativeHandle,qv);
        end

        % reference：引用管理器及其步态表
        function reference_setTargetTrajectories(obj,times,states,inputs)
            ocs2_binding_mex('reference_setTargetTrajectories',obj.NativeHandle,times,states,inputs);
        end
        function reference_getGaitSchedule_updateModeSchedule(obj,eventTimes,modes)
            % 原样传递；GaitSchedule 所需的末尾标记由 MATLAB 调用者提供。
            ocs2_binding_mex('reference_getGaitSchedule_updateModeSchedule',obj.NativeHandle,eventTimes,modes);
        end
        function reference_getGaitSchedule_disableExtension(obj)
            % 绑定辅助：通过 insertModeSequenceTemplate 安装空循环模板。
            % disableExtension 不是上游方法；外部负责完整接触时间表。
            ocs2_binding_mex('reference_getGaitSchedule_disableExtension',obj.NativeHandle);
        end
        function reference=reference_getSnapshot(obj)
            % 绑定辅助：合并 getTargetTrajectories 和 getModeSchedule 的结果。
            reference=ocs2_binding_mex('reference_getSnapshot',obj.NativeHandle);
        end

        % mrt：原生方法名称保持与 OCS2 一致
        function mrt_resetMpcNode(obj,times,states,inputs)
            ocs2_binding_mex('mrt_resetMpcNode',obj.NativeHandle,times,states,inputs);
        end
        function mrt_setCurrentObservation(obj,time,state,input,mode)
            % 同时同步 reference.setMeasuredMode，保持原有观测语义。
            ocs2_binding_mex('mrt_setCurrentObservation',obj.NativeHandle,time,state,input,mode);
        end
        function mrt_advanceMpc(obj)
            ocs2_binding_mex('mrt_advanceMpc',obj.NativeHandle);
        end
        function changed=mrt_updatePolicy(obj)
            changed=ocs2_binding_mex('mrt_updatePolicy',obj.NativeHandle);
        end
        function [state,input,mode]=mrt_evaluatePolicy(obj,time,measuredState)
            [state,input,mode]=ocs2_binding_mex('mrt_evaluatePolicy',obj.NativeHandle,time,measuredState);
        end

        % 诊断与绑定层复制统计
        function enableDiagnostics(obj)
            ocs2_binding_mex('enableDiagnostics',obj.NativeHandle);
        end
        function batch=diagnostics_drain(obj)
            batch=ocs2_binding_mex('diagnostics_drain',obj.NativeHandle);
        end
        function diagnostics_observation(obj,tick)
            ocs2_binding_mex('diagnostics_observation',obj.NativeHandle,tick);
        end
        function diagnostics_trace(obj,event,time)
            ocs2_binding_mex('diagnostics_trace',obj.NativeHandle,event,time);
        end
        function diagnostics_reference(obj,time,command)
            ocs2_binding_mex('diagnostics_reference',obj.NativeHandle,time,command);
        end
        function stats=copyStats(obj)
            % 各项为 [显式搬运次数, 元素总数]；不包括 OCS2 内部和 MATLAB 分配。
            stats=ocs2_binding_mex('copyStats',obj.NativeHandle);
        end

    end
end
