classdef PlannerBackend < handle
    %PLANNERBACKEND MATLAB 拥有参考协议、求解阶段和策略缓存。
    % 原生 Problem 只保管模型、SQP/MRT、原生计算和只读诊断对象。
    properties (Access=private)
        problem
        dimensions
        settings
        wholeBody
        diagnosticsEnabled
        token=[]
        state=[]
        policy=[]
        updated=false
        requested=false
        started=false
        failed=false
        reference=[]
        sequence=0
        consumedSequence=0
        consumedTick=-1
        consumption=[]
    end
    methods
        function obj=PlannerBackend(config)
            obj.problem=ocs2.Problem(config);
            obj.wholeBody=strcmp(config.formulation,'whole_body');
            obj.settings=obj.problem.modelSettings();
            obj.dimensions=[obj.problem.Dimensions(1:2);obj.settings.joints];
            obj.diagnosticsEnabled=~isfield(config,'diagnosticsEnabled') || config.diagnosticsEnabled;
            obj.problem.createSolver();obj.problem.reference_getGaitSchedule_disableExtension();
            if obj.diagnosticsEnabled,obj.problem.enableDiagnostics();end
        end
        function delete(obj)
            if ~isempty(obj.problem),delete(obj.problem);obj.problem=[];end
        end
        function varargout=execute(obj,command,varargin)
            switch command
                case 'dimensions'
                    varargout{1}=obj.dimensions;
                case 'convert_observation'
                    varargout{1}=obj.problem.convertObservation(varargin{1});
                case 'install_reference'
                    obj.installReference(varargin{1});
                case 'install_observation'
                    varargout{1}=obj.installObservation(varargin{1},varargin{2});
                case 'update_mpc'
                    varargout{1}=obj.update(varargin{1},varargin{2});
                case 'query_policy'
                    varargout{1}=obj.query(varargin{1});
                case 'diagnostics'
                    batch=obj.problem.diagnostics_drain();
                    if ~isempty(obj.consumption)
                        row=struct('name','adapter_consumption', ...
                            'columns',{{'session','epoch','sequence','control_tick','canonical_control_time', ...
                            'target_count','event_count','event_mode_max_error','target_max_error', ...
                            'legacy_filter_calls','legacy_gait_decisions'}},'values',{num2cell(obj.consumption)});
                        batch.tables(end+1)=row;obj.consumption=[];
                    end
                    varargout{1}=batch;
                otherwise
                    error('G1:PlannerMex','Unknown planner command: %s',command);
            end
        end
    end
    methods (Access=private)
        function installReference(obj,a)
            nx=obj.dimensions(1);nu=obj.dimensions(2);
            vector(a,149+3*(nx+nu));a=a(:);
            require(a(2)==0 || a(2)==1,'Invalid reference publication flag');
            if a(2)==0,return;end
            integer(a(1),0,flintmax);integer(a(3),1,3);
            require(a(3)==1 || a(3)==3,'Invalid target count');
            if ~isempty(obj.reference) && obj.reference.tick==a(1),return;end
            offset=6+3*(nx+nu);count=a(offset+1);integer(count,1,64);
            modes=a(offset+66:offset+130);
            require(all(modes>=0 & modes<=3 & modes==floor(modes)),'Invalid contact mode');
            r=struct('tick',a(1),'sequence',obj.sequence+1,'times',a(4:3+a(3)), ...
                'states',reshape(a(7:6+3*nx),nx,3), ...
                'inputs',reshape(a(7+3*nx:offset),nu,3), ...
                'events',a(offset+2:offset+1+count),'modes',modes(1:count+1), ...
                'terminal',a(offset+131),'swing',a(offset+132:offset+139), ...
                'filtered',a(offset+140:offset+143));
            r.states=r.states(:,1:a(3));r.inputs=r.inputs(:,1:a(3));
            obj.sequence=r.sequence;obj.reference=r;
        end
        function token=installObservation(obj,x,identity)
            vector(x,obj.dimensions(1));vector(identity,2);
            tick=identity(1);integer(tick,0,flintmax);
            if ~isempty(obj.token) && tick==obj.token(1)
                require(identity(2)==obj.token(2),'Repeated observation changed time');
                token=obj.token;return;
            end
            if isempty(obj.token)
                require(tick==0,'First physical tick must be zero');
            else
                require(~isempty(obj.policy),'Query prepared observation before advancing tick');
                require(tick==obj.token(1)+1,'Nonsequential physical tick');
            end
            obj.token=[tick;identity(2);obj.problem.Dimensions(3)];
            obj.state=x(:);obj.policy=[];obj.updated=false;token=obj.token;
        end
        function token=update(obj,token,request)
            obj.validateToken(token);vector(request,1);
            require(request==0 || request==1,'MPC request must be zero or one');
            if obj.updated
                require(logical(request)==obj.requested,'Repeated MPC stage changed request');return;
            end
            require(~obj.failed,'Previous solve failed; close and initialize a fresh backend');
            require(obj.started || request==1,'Initial observation requires a solve');
            t=obj.token(2);nu=obj.dimensions(2);
            obj.problem.mrt_setCurrentObservation(t,obj.state,zeros(nu,1),3);
            if obj.diagnosticsEnabled
                obj.problem.diagnostics_observation(obj.token(1));obj.problem.diagnostics_trace('observation_arrived',t);
            end
            if request
                try
                    obj.validateReference(t);
                    if ~obj.started
                        initial=obj.state;
                        if obj.wholeBody
                            initial(end-(6+obj.settings.joints)+1:end)=0;initial(5:6)=0;
                        end
                        obj.problem.mrt_resetMpcNode(t,initial,zeros(nu,1));
                    end
                    r=obj.reference;
                    obj.problem.reference_setTargetTrajectories(r.times,r.states,r.inputs);
                    obj.problem.reference_getGaitSchedule_updateModeSchedule([r.events;r.terminal],[r.modes;3]);
                    if obj.diagnosticsEnabled
                        obj.problem.diagnostics_reference(t,r.filtered);obj.problem.diagnostics_trace('solver_started',t);
                    end
                    obj.problem.mrt_advanceMpc();
                    if obj.diagnosticsEnabled,obj.problem.diagnostics_trace('solver_finished',t);end
                    require(obj.problem.mrt_updatePolicy(),'Requested solve did not install a policy');
                    if obj.diagnosticsEnabled,obj.problem.diagnostics_trace('policy_installed',t);end
                    obj.verifyConsumption(t);obj.started=true;
                catch exception
                    obj.failed=true;rethrow(exception);
                end
            end
            obj.updated=true;obj.requested=logical(request);
        end
        function policy=query(obj,token)
            obj.validateToken(token);
            require(~obj.failed && obj.updated,'Complete MPC update before querying policy');
            if isempty(obj.policy)
                t=obj.token(2)+.005;
                [x,u,mode]=obj.problem.mrt_evaluatePolicy(t,obj.state);
                obj.policy=[x;u;mode;t];
            end
            policy=obj.policy;
        end
        function validateToken(obj,token)
            vector(token,3);
            require(~isempty(obj.token) && isequal(token(:),obj.token),'Planner tick identity mismatch');
        end
        function validateReference(obj,t)
            r=obj.reference;
            require(~isempty(r) && r.tick==obj.token(1),'Reference tick does not match solver observation');
            require(r.sequence==obj.consumedSequence+1 && r.tick>obj.consumedTick,'Reference order rejected');
            require(all(diff(r.times)>0) && all(diff(r.events)>=0),'Reference times not ordered');
            require(r.terminal>r.events(end) && r.terminal>t+2*obj.settings.horizon,'Terminal marker inside reference window');
            require(isequal(r.swing,obj.settings.swing),'Swing parameters differ from loaded problem');
        end
        function verifyConsumption(obj,t)
            r=obj.reference;actual=obj.problem.reference_getSnapshot();
            require(isequal(actual.events,r.events) && isequal(actual.modes,r.modes),'Adapter altered authoritative schedule');
            require(isequal(actual.times,r.times) && isequal(actual.states,r.states) && ...
                isequal(actual.inputs,r.inputs),'Adapter altered authoritative target');
            obj.consumedSequence=r.sequence;obj.consumedTick=r.tick;
            if obj.diagnosticsEnabled
                obj.consumption(end+1,:)=[1,0,r.sequence,r.tick,t,numel(r.times),numel(r.events),0,0,0,0];
            end
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
