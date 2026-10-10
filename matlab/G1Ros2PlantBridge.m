classdef G1Ros2PlantBridge < handle
%G1ROS2PLANTBRIDGE Bounded sequence-matched topics, simulation owns time.
% Lifecycle: DISCONNECTED=0 INITIALIZING=1 READY=2 RUNNING=3 STALE=4
% RESETTING=5 ERROR=6. Each tick clears action; invalid/timeout => zero gains/ff.
 properties
  Config
  Session uint64
  Sequence uint64=0
  Endpoint uint64=0
  ActionSequence uint64=0
  Node
  Publisher
  Subscriber
  Observer
  Observations cell={}
  LastState=[]
  Inbox=[]
  Lifecycle double=1
  LastTick double=-1
  LastOutput double=zeros(163,1)
  WireNames string=string.empty
  WireIndices double=[]
  RetiredEndpoints uint64=uint64.empty
  Events cell={}
  Counts double=zeros(1,6) % drop, out of order, stale, timeout, restart, reject
  Accepted double=0
 end
 methods
  function obj=G1Ros2PlantBridge(cfg)
   obj.Config=cfg;
   if isfield(cfg,'transportEnabled') && ~cfg.transportEnabled,obj.Lifecycle=0;return;end
   g3a_load_messages();
   obj.Session=uint64(posixtime(datetime('now','TimeZone','UTC'))*1e6)*uint64(1024)+uint64(randi(1023));
   obj.Node=ros2node(['/g1_simscape_' num2str(randi(1e8))],cfg.domainId);
   qos={'History','keeplast','Depth',1,'Reliability','reliable','Durability','volatile'};
   obj.Publisher=ros2publisher(obj.Node,'/g1_bridge/state','g1_plant_bridge_msgs/PlantState',qos{:});
   obj.Subscriber=ros2subscriber(obj.Node,'/g1_bridge/action','g1_plant_bridge_msgs/RobotJointAction',@(msg)obj.enqueue(msg),qos{:});
   obj.Observer=ros2subscriber(obj.Node,'/g1_bridge/observation','g1_plant_bridge_msgs/BridgeObservation',@(msg)obj.observe(msg),qos{:});
  end
  function observe(obj,msg)
   obj.Observations{end+1}=msg;
  end
  function enqueue(obj,msg)
   obj.Inbox=msg; % one pending action, no unbounded MATLAB callback queue
  end
  function [indices,reason]=mapping(obj,msg)
   indices=[];reason='';names=string(msg.joint_names(:));
   if numel(names)~=29,reason='joint_count';return;end
   if numel(unique(names))~=29,reason='duplicate_joint';return;end
   [found,indices]=ismember(obj.Config.jointNames(:),names);
   if ~all(found),indices=[];reason='missing_unknown_joint';end
  end
  function [values,reason]=validateAction(obj,msg,sequence)
   values=zeros(145,1);reason='';
   if msg.session_id~=obj.Session,reason='wrong_session';return;end
   if msg.state_sequence~=sequence,reason='wrong_state_sequence';return;end
   if msg.endpoint_id==0 || any(obj.RetiredEndpoints==msg.endpoint_id),reason='retired_endpoint';return;end
   newEndpoint=obj.Endpoint~=msg.endpoint_id;
   if msg.sequence==0 || (~newEndpoint && msg.sequence<=obj.ActionSequence),reason='action_sequence_rollback';return;end
   if newEndpoint
    [indices,reason]=obj.mapping(msg);if ~isempty(reason),return;end
   else
    if ~isequal(string(msg.joint_names(:)),obj.WireNames),reason='mapping_changed';return;end
    indices=obj.WireIndices;
   end
   for k=1:5
    v=msg.(obj.Config.actionFields{k});
    if numel(v)~=29 || any(~isfinite(v(:))),reason='invalid_numeric_action';return;end
    if any(k==[3 4]) && any(v(:)<0),reason='negative_gain';return;end
    values((k-1)*29+(1:29))=v(indices);
   end
   if newEndpoint
    if obj.Endpoint~=0,obj.RetiredEndpoints(end+1)=obj.Endpoint;obj.Counts(5)=obj.Counts(5)+1;end
    obj.Endpoint=msg.endpoint_id;obj.WireNames=string(msg.joint_names(:));obj.WireIndices=indices;
   elseif msg.sequence>obj.ActionSequence+1
    obj.Counts(1)=obj.Counts(1)+double(msg.sequence-obj.ActionSequence-1);
   end
   obj.ActionSequence=msg.sequence;
  end
  function record(obj,event,time,msg,sendWall,receiveWall,latency,age)
   obj.Events{end+1}=struct('event',event,'session_id',obj.Session, ...
    'plant_sequence',obj.Sequence,'sim_time',time,'action',msg, ...
    'send_wall_time',sendWall,'receive_wall_time',receiveWall, ...
    'latency_s',latency,'message_age_s',age,'counts',obj.Counts,'state',obj.LastState);
  end
  function output=exchange(obj,u)
   assert(numel(u)==74);
   if obj.Lifecycle==0,output=zeros(163,1);return;end
   t=u(1);tick=round(t/obj.Config.actionTs);
   if tick==obj.LastTick,output=obj.LastOutput;return;end
   if tick<obj.LastTick
    obj.Lifecycle=6;output=zeros(163,1);output(146)=6;
    obj.record('simulation_rollback',t,[],0,0,0,0);obj.LastOutput=output;return;
   end
   obj.LastTick=tick;obj.Sequence=uint64(tick);
   if isfield(obj.Config,'beforeTick'),obj.Config.beforeTick(obj,tick);end
   state=ros2message(obj.Publisher);state.session_id=obj.Session;state.sequence=obj.Sequence;
   state.sim_time=t;state.base_position_world=u(2:4);state.base_orientation_wxyz=u(5:8);
   state.base_linear_velocity_local=u(9:11);state.base_angular_velocity_local=u(12:14);
   state.joint_position=u(15:43);state.joint_velocity=u(44:72);
   state.joint_names=cellstr(obj.Config.jointNames(:));state.measured_contact_flags=logical(u(73:74));
   sendWall=posixtime(datetime('now','TimeZone','UTC'));state.stamp=g3a_stamp(sendWall);
   obj.LastState=state;send(obj.Publisher,state);
   waitStart=tic;timeout=obj.Config.replyTimeout;
   if obj.Accepted==0 && obj.Counts(4)==0,timeout=obj.Config.initialTimeout;end
   action=zeros(145,1);receiveWall=0;latency=0;age=0;coherent=false;malformed=false;
   while toc(waitStart)<timeout
    drawnow;
    if toc(waitStart)>=timeout,break;end
    if ~isempty(obj.Inbox)
     msg=obj.Inbox;obj.Inbox=[];receiveWall=posixtime(datetime('now','TimeZone','UTC'));
     [candidate,reason]=obj.validateAction(msg,obj.Sequence);
     latency=receiveWall-sendWall;
     age=receiveWall-(double(msg.stamp.sec)+double(msg.stamp.nanosec)*1e-9);
     if isempty(reason)
      action=candidate;coherent=true;obj.Accepted=obj.Accepted+1;obj.Lifecycle=3;
      obj.record('accepted',t,msg,sendWall,receiveWall,latency,age);break;
     end
     obj.Counts(6)=obj.Counts(6)+1;
     if any(strcmp(reason,{'wrong_session','wrong_state_sequence','retired_endpoint'})),obj.Counts(3)=obj.Counts(3)+1;
     elseif strcmp(reason,'action_sequence_rollback'),obj.Counts(2)=obj.Counts(2)+1;
     else,malformed=true;end
     obj.record(reason,t,msg,sendWall,receiveWall,latency,age);
    end
    pause(0.001);
   end
   elapsed=toc(waitStart);
   if ~coherent
    obj.Counts(4)=obj.Counts(4)+1;obj.Lifecycle=4;
    if malformed,obj.Lifecycle=6;end
    obj.record('timeout_fallback',t,[],sendWall,receiveWall,latency,age);
   end
   diag=[obj.Lifecycle;double(obj.Sequence);double(obj.ActionSequence);double(obj.Sequence);t; ...
    sendWall;receiveWall;latency;age;obj.Counts(1:5)';obj.Accepted;obj.Counts(6);double(coherent);elapsed];
   output=[action;diag];obj.LastOutput=output;
  end
  function delete(obj)
   if ~isempty(obj.Node),delete(obj.Node);end
  end
 end
end
