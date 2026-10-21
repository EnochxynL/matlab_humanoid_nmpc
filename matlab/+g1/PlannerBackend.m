classdef PlannerBackend < handle
    % 一个控制步显式接收观测和参考，顺序完成提交、可选求解及策略查询。
    properties (Access=private)
        problem
        dimensions
        settings
        wholeBody
        diagnosticsEnabled
        started=false
        failed=false
        lastTick=[]
        lastArguments={}
        lastPolicy=[]
        solveCount=0
        localTables=struct()
        wallClock=[]
        clockMetadata=[]
        observationMode=3
    end
    methods
        function obj=PlannerBackend(config)
            obj.problem=ocs2.Problem(config);
            obj.wholeBody=strcmp(config.formulation,'whole_body');
            obj.settings=obj.problem.modelSettings();
            obj.dimensions=[obj.problem.Dimensions(1:2);obj.settings.joints];
            obj.diagnosticsEnabled=~isfield(config,'diagnosticsEnabled') || config.diagnosticsEnabled;
            obj.problem.createSolver();obj.problem.reference_getGaitSchedule_disableExtension();
            if obj.diagnosticsEnabled
                obj.problem.enableDiagnostics();
                obj.wallClock=tic;
                obj.clockMetadata=struct('table','timing_trace', ...
                    'column','wall_monotonic_time_s','clock','MATLAB tic/toc', ...
                    'origin','PlannerBackend diagnostic initialization','unit','seconds', ...
                    'scope','session relative; do not subtract from native wall timestamps', ...
                    'policy_consumed','after native policy query and diagnostic evaluation return');
                obj.defineTable('timing_trace',{'event','control_time_s','wall_monotonic_time_s'});
                obj.defineTable('reference',{'control_time_s','v_x','v_y','height','yaw_rate'});
                obj.defineTable('gait_queries',{'observation_time_s','query_control_time_s','observation_mode','policy_mode'});
                obj.defineTable('logging_load',{'observation_time_s','repeat','knot_count','first_state_norm'});
                obj.defineTable('adapter_consumption',{'session','epoch','sequence','control_tick', ...
                    'canonical_control_time','target_count','event_count','event_mode_max_error', ...
                    'target_max_error','legacy_filter_calls','legacy_gait_decisions'});
            end
        end
        function delete(obj)
            if ~isempty(obj.problem),delete(obj.problem);obj.problem=[];end
        end
        function varargout=execute(obj,command,varargin)
            switch command
                case 'dimensions'
                    varargout{1}=obj.dimensions;
                case 'mpc_period'
                    varargout{1}=obj.problem.Dimensions(3);
                case 'convert_observation'
                    varargout{1}=obj.problem.convertObservation(varargin{1});
                case 'step'
                    varargout{1}=obj.step(varargin{:});
                case 'diagnostics'
                    batch=obj.problem.diagnostics_drain();
                    names=fieldnames(obj.localTables);
                    for k=1:numel(names)
                        name=names{k};batch.tables(end+1)=obj.localTables.(name);
                        obj.localTables.(name).values=cell(0,numel(obj.localTables.(name).columns));
                    end
                    batch.clockMetadata=obj.clockMetadata;obj.clockMetadata=[];
                    varargout{1}=batch;
                otherwise
                    error('G1:PlannerMex','Unknown planner command: %s',command);
            end
        end
    end
    methods (Access=private)
        function policy=step(obj,state,reference,identity,request)
            vector(state,obj.dimensions(1));vector(identity,2);vector(request,1);
            tick=identity(1);t=identity(2);integer(tick,0,flintmax);
            require(request==0 || request==1,'MPC request must be zero or one');
            require(~obj.failed,'Previous solve failed; close and initialize a fresh backend');
            stepArguments={state(:),reference(:),identity(:),request};
            if ~isempty(obj.lastTick) && tick==obj.lastTick
                require(isequal(stepArguments,obj.lastArguments),'Repeated planner step changed inputs');
                policy=obj.lastPolicy;return;
            end
            if isempty(obj.lastTick)
                require(tick==0 && request==1,'First planner step requires tick zero and a solve');
            else
                require(tick==obj.lastTick+1,'Nonsequential physical tick');
            end
            % 参考仅在本步求解时使用，不预装到后端，也不跨步缓存参考包。
            if request,r=obj.decodeReference(reference,tick,t);end
            try
                obj.problem.mrt_setCurrentObservation(t,state(:),zeros(obj.dimensions(2),1),obj.observationMode);
                if obj.diagnosticsEnabled,obj.trace('observation_arrived',t);end
                if request
                    if ~obj.started
                        initial=state(:);
                        if obj.wholeBody
                            initial(end-(6+obj.settings.joints)+1:end)=0;initial(5:6)=0;
                        end
                        obj.problem.mrt_resetMpcNode(t,initial,zeros(obj.dimensions(2),1));
                    end
                    obj.problem.reference_setTargetTrajectories(r.times,r.states,r.inputs);
                    obj.problem.reference_getGaitSchedule_updateModeSchedule([r.events;r.terminal],[r.modes;3]);
                    if obj.diagnosticsEnabled
                        obj.problem.diagnostics_observation(tick);
                        obj.record('reference',num2cell([t;r.filtered]'));
                        obj.trace('solver_started',t);
                    end
                    obj.problem.mrt_advanceMpc();
                    if obj.diagnosticsEnabled,obj.trace('solver_finished',t);end
                    require(obj.problem.mrt_updatePolicy(),'Requested solve did not install a policy');
                    if obj.diagnosticsEnabled,obj.trace('policy_installed',t);end
                    obj.verifyReference(r,tick,t);obj.started=true;
                end
                queryTime=t+.005;
                [x,u,mode]=obj.problem.mrt_evaluatePolicy(queryTime,state(:));
                policy=[x;u;mode;queryTime];
                if obj.diagnosticsEnabled
                    obj.record('gait_queries',{t,queryTime,obj.observationMode,mode});
                    obj.trace('policy_consumed',t);
                end
            catch exception
                obj.failed=true;rethrow(exception);
            end
            obj.lastTick=tick;obj.lastArguments=stepArguments;obj.lastPolicy=policy;
        end
        function r=decodeReference(obj,a,tick,t)
            nx=obj.dimensions(1);nu=obj.dimensions(2);
            vector(a,149+3*(nx+nu));a=a(:);
            require(a(1)==tick && a(2)==1,'Requested solve requires a reference for the current tick');
            integer(a(3),1,3);require(a(3)==1 || a(3)==3,'Invalid target count');
            offset=6+3*(nx+nu);count=a(offset+1);integer(count,1,64);
            modes=a(offset+66:offset+66+count);
            require(all(modes>=0 & modes<=3 & modes==floor(modes)),'Invalid contact mode');
            r=struct('times',a(4:3+a(3)),'states',reshape(a(7:6+3*nx),nx,3), ...
                'inputs',reshape(a(7+3*nx:offset),nu,3),'events',a(offset+2:offset+1+count), ...
                'modes',modes,'terminal',a(offset+131),'swing',a(offset+132:offset+139), ...
                'filtered',a(offset+140:offset+143));
            r.states=r.states(:,1:a(3));r.inputs=r.inputs(:,1:a(3));
            require(all(diff(r.times)>0) && all(diff(r.events)>=0),'Reference times not ordered');
            require(r.terminal>r.events(end) && r.terminal>t+2*obj.settings.horizon,'Terminal marker inside reference window');
            require(isequal(r.swing,obj.settings.swing),'Swing parameters differ from loaded problem');
        end
        function verifyReference(obj,r,tick,t)
            actual=obj.problem.reference_getSnapshot();
            require(isequal(actual.events,r.events) && isequal(actual.modes,r.modes),'Adapter altered authoritative schedule');
            require(isequal(actual.times,r.times) && isequal(actual.states,r.states) && ...
                isequal(actual.inputs,r.inputs),'Adapter altered authoritative target');
            obj.solveCount=obj.solveCount+1;
            if obj.diagnosticsEnabled
                obj.record('adapter_consumption',num2cell([1,0,obj.solveCount,tick,t,numel(r.times),numel(r.events),0,0,0,0]));
            end
        end
        function defineTable(obj,name,columns)
            obj.localTables.(name)=struct('name',name,'columns',{columns},'values',{cell(0,numel(columns))});
        end
        function record(obj,name,values)
            obj.localTables.(name).values(end+1,:)=values;
        end
        function trace(obj,event,time)
            % 仅本会话 elapsed time；与 native steady_clock 的时间原点不同。
            obj.record('timing_trace',{event,time,toc(obj.wallClock)});
        end
    end
end
function vector(value,count)
require(isa(value,'double') && isreal(value) && ~issparse(value) && ...
    numel(value)==count && all(isfinite(value(:))),'Invalid numeric argument');
end
function integer(value,minimum,maximum)
require(value>=minimum && value<=maximum && value==floor(value),'Invalid integer argument');
end
function require(condition,message)
if ~condition,error('G1:PlannerMex','%s',message);end
end
