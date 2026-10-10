function build_inprocess_model()
% Reproducible model construction; original plant/JointDrive remain intact.
root=fileparts(mfilename('fullpath'));cd(root);addpath(fullfile(root,'matlab'));addpath(fullfile(root,'models'));
g1c_define_buses();task5a_define_buses();task5b_define_buses();
src='g1_wb_mpc_wave_c_foot_cost';open_system(fullfile(root,'models',[src '.slx']));
m='g1_inprocess_walking';assert(~bdIsLoaded(m),'Close the new model before rebuilding it.');
new_system(m);cs=copy(getActiveConfigSet(src));attachConfigSet(m,cs,true);setActiveConfigSet(m,cs.Name);
set_param(m,'InitFcn','g1c_define_buses;task5a_define_buses;', ...
 'StartFcn','','StopFcn','','PreLoadFcn','','PostLoadFcn','','CloseFcn','', ...
 'SimMechanicsOpenEditorOnUpdate','on');
w=get_param(src,'ModelWorkspace');nw=get_param(m,'ModelWorkspace');vars=w.whos;
for k=1:numel(vars),nw.assignin(vars(k).name,w.getVariable(vars(k).name));end
nw.assignin('task5a_standing',0);
cfg=g1b_config();M=zeros(29,23);for k=1:23,M(cfg.mpcIndices(k),k)=1;end
nw.assignin('g1_mpc_to_plant',M);kp=100*ones(29,1);kd=ones(29,1);kp(cfg.mpcIndices)=1200;kd(cfg.mpcIndices)=10;nw.assignin('g1_kp',kp);nw.assignin('g1_kd',kd);
copyblock(1,'PhysicalPlant',[1060 220 1220 360]);copyblock(10,'JointDrive',[825 215 975 345]);
add_block('simulink/User-Defined Functions/MATLAB Function',[m '/StatePreparation'],'Position',[180 75 300 130]);
sc=sfroot().find('-isa','Stateflow.EMChart','Path',[m '/StatePreparation']);
code=fileread(fullfile(root,'matlab/task5a_observation.m'));
code=strrep(code,'function s=task5a_observation(p,session)',sprintf('function s=prepareObservation(p)\nsession=uint64(1);'));sc.Script=code;
sc.find('-isa','Stateflow.Data','Name','p').DataType='Bus: PlantState';
sc.find('-isa','Stateflow.Data','Name','s').DataType='Bus: Task5AReferenceState';
add_block('built-in/Subsystem',[m '/Command'],'Position',[180 160 300 215]);
command=[m '/Command'];add_block('built-in/Inport',[command '/Observation'],'OutDataTypeStr','Bus: Task5AReferenceState','Position',[0 30 30 44]);
add_block('built-in/Inport',[command '/Standing'],'Port','2','Position',[0 130 30 144]);
add_block('simulink/Signal Routing/Bus Selector',[command '/Tick'],'OutputSignals','tick','Position',[70 25 75 60]);
add_block('simulink/Logic and Bit Operations/Compare To Constant',[command '/CommandStarts'],'relop','>=','const','uint64(250)','Position',[130 25 225 60]);
add_block('simulink/Logic and Bit Operations/Logical Operator',[command '/WalkingEnabled'],'Operator','NOT','Inputs','1','Position',[130 115 225 150]);
add_block('simulink/Logic and Bit Operations/Logical Operator',[command '/ForwardEnabled'],'Operator','AND','Inputs','2','Position',[280 55 330 115]);
add_block('simulink/Signal Attributes/Data Type Conversion',[command '/ForwardDouble'],'OutDataTypeStr','double','Position',[365 65 425 100]);
add_block('simulink/Math Operations/Gain',[command '/ForwardCommand'],'Gain','.1','Position',[465 65 525 100]);
add_block('simulink/Sources/Constant',[command '/LateralCommand'],'Value','0','Position',[465 145 525 175]);
add_block('simulink/Sources/Constant',[command '/HeightCommand'],'Value','.8','Position',[465 210 525 240]);
add_block('simulink/Sources/Constant',[command '/YawCommand'],'Value','0','Position',[465 275 525 305]);
add_block('simulink/Signal Routing/Bus Creator',[command '/LocomotionCommand'],'Inputs','4','OutDataTypeStr','Bus: Task5ALocomotionCommand','NonVirtualBus','on','Position',[600 65 605 290]);
add_block('built-in/Outport',[command '/Command'],'Position',[670 165 700 179]);
links={'Observation/1','Tick/1';'Tick/1','CommandStarts/1';'Standing/1','WalkingEnabled/1';'CommandStarts/1','ForwardEnabled/1';'WalkingEnabled/1','ForwardEnabled/2';'ForwardEnabled/1','ForwardDouble/1';'ForwardDouble/1','ForwardCommand/1';'LocomotionCommand/1','Command/1'};
for k=1:size(links,1),add_line(command,links{k,1},links{k,2},'autorouting','on');end
ports={'ForwardCommand','LateralCommand','HeightCommand','YawCommand'};fields={'vx','vy','height','yaw_rate'};
for k=1:4,h=add_line(command,[ports{k} '/1'],sprintf('LocomotionCommand/%d',k),'autorouting','on');set_param(h,'Name',fields{k});end

copyblock(52,'ReferenceGait',[360 70 550 150]);copyblock(54,'ReferencePacket',[360 175 550 245]);
set_param([m '/ReferenceGait'],'ModelName','g1_inprocess_reference');
add_block('simulink/Sources/Constant',[m '/Standing'],'Value','task5a_standing','Position',[65 170 125 200]);
add_block('simulink/Discrete/Zero-Order Hold',[m '/StateAtControlTick'],'SampleTime','.002','Position',[40 70 120 120]);
add_block('simulink/Signal Routing/Bus Selector',[m '/ObservationFields'],'OutputSignals','sim_time,base_position_world,base_orientation_wxyz,base_linear_velocity_local,base_angular_velocity_local,q,qd','Position',[180 310 185 460]);
add_block('simulink/Signal Routing/Bus Selector',[m '/MeasuredContact'],'OutputSignals','Left.in_contact,Right.in_contact','Position',[180 490 185 540]);
add_block('simulink/Signal Routing/Mux',[m '/ObservationVector'],'Inputs','9','Position',[335 310 340 550]);
add_block('simulink/User-Defined Functions/MATLAB Function',[m '/OCS2PlannerMEX'],'Position',[590 140 735 220]);
c=sfroot().find('-isa','Stateflow.EMChart','Path',[m '/OCS2PlannerMEX']);
c.Script=sprintf(['function plan=planner(state,reference)\n%%#codegen\n' ...
 'coder.extrinsic(''g1_ocp_mex'');\nplan=zeros(118,1);\n' ...
 'g1_ocp_mex(''reference'',reference);\nplan=g1_ocp_mex(''step'',state);\nend\n']);
add_block('built-in/Subsystem',[m '/ActionGeneration'],'Position',[805 70 970 140]);
a=[m '/ActionGeneration'];add_block('built-in/Inport',[a '/PolicyAndID'],'PortDimensions','118');
for k=1:3
 names={'JointPosition','JointVelocity','FeedForwardTorque'};indices={'7:29','36:58','94:116'};
 add_block('simulink/Signal Routing/Selector',[a '/' names{k}],'NumberOfDimensions','1','IndexOptionArray',{'Index vector (dialog)'},'Indices',indices{k},'InputPortWidth','118','Position',[95 30+70*k 175 65+70*k]);
 add_block('simulink/Math Operations/Gain',[a '/Map' names{k}],'Gain','g1_mpc_to_plant','Multiplication','Matrix(K*u)','Position',[240 30+70*k 320 65+70*k]);
 add_line(a,'PolicyAndID/1',[names{k} '/1'],'autorouting','on');add_line(a,[names{k} '/1'],['Map' names{k} '/1'],'autorouting','on');
end
add_block('simulink/Sources/Constant',[a '/PositionGains'],'Value','g1_kp','Position',[240 320 320 350]);add_block('simulink/Sources/Constant',[a '/VelocityGains'],'Value','g1_kd','Position',[240 380 320 410]);
add_block('simulink/Signal Routing/Bus Creator',[a '/RobotJointAction'],'Inputs','5','OutDataTypeStr','Bus: RobotJointAction','NonVirtualBus','on','Position',[390 100 395 390]);
ports={'MapJointPosition','MapJointVelocity','PositionGains','VelocityGains','MapFeedForwardTorque'};fields=cfg.actionFields;
for k=1:5,h=add_line(a,[ports{k} '/1'],sprintf('RobotJointAction/%d',k),'autorouting','on');set_param(h,'Name',fields{k});end
add_block('built-in/Outport',[a '/Action'],'OutDataTypeStr','Bus: RobotJointAction','Position',[475 240 505 254]);add_line(a,'RobotJointAction/1','Action/1');
wire('PhysicalPlant/1','StateAtControlTick/1');wire('StateAtControlTick/1','StatePreparation/1');wire('StateAtControlTick/1','ObservationFields/1');
wire('StatePreparation/1','ReferenceGait/1');wire('StatePreparation/1','Command/1');wire('Standing/1','Command/2');wire('Command/1','ReferenceGait/2');
wire('ReferenceGait/1','ReferencePacket/1');wire('ReferencePacket/1','OCS2PlannerMEX/2');
wire('PhysicalPlant/3','MeasuredContact/1');
for k=1:7,wire(sprintf('ObservationFields/%d',k),sprintf('ObservationVector/%d',k));end
for k=1:2
 name=sprintf('ContactDouble%d',k);add_block('simulink/Signal Attributes/Data Type Conversion',[m '/' name],'OutDataTypeStr','double','Position',[235 465+45*k 285 490+45*k]);
 wire(sprintf('MeasuredContact/%d',k),[name '/1']);wire([name '/1'],sprintf('ObservationVector/%d',k+7));
end
add_block('simulink/Discrete/Zero-Order Hold',[m '/PlannerObservationHold'],'SampleTime','.002','Position',[425 330 485 370]);
wire('ObservationVector/1','PlannerObservationHold/1');wire('PlannerObservationHold/1','OCS2PlannerMEX/1');wire('OCS2PlannerMEX/1','ActionGeneration/1');wire('ActionGeneration/1','JointDrive/1');
wire('PhysicalPlant/4','JointDrive/2');wire('PhysicalPlant/5','JointDrive/3');wire('JointDrive/1','PhysicalPlant/1');
outputs={'PlantState','RobotJointAction','DriveDiagnostics','WorldDiagnostics','ContactDiagnostics','PolicyAndID','ReferenceGaitDiagnostics'};
sources={'PhysicalPlant/1','JointDrive/2','JointDrive/3','PhysicalPlant/2','PhysicalPlant/3','OCS2PlannerMEX/1','ReferenceGait/2'};
for k=1:numel(outputs)
 add_block('built-in/Outport',[m '/' outputs{k}],'Port',num2str(k),'Position',[1320 30+55*k 1350 44+55*k]);wire(sources{k},[outputs{k} '/1']);
end
set_param([m '/ReferenceGaitDiagnostics'],'OutDataTypeStr','Bus: Task5AReferenceGait','BusOutputAsStruct','on');
% Replace file publication with a direct flattened signal; keep exact field order.
p=[m '/ReferencePacket'];writer=[p '/AtomicPacketWriter'];lines=get_param(writer,'LineHandles');delete_line(lines.Inport);delete_line(lines.Outport);delete_block(writer);
outport=find_system(p,'SearchDepth',1,'BlockType','Outport');lh=get_param(outport{1},'LineHandles');if lh.Inport~=-1,delete_line(lh.Inport);end %#ok<FNDSB>
add_line(p,'TransportEnvelope/1',[get_param(outport{1},'Name') '/1']);
lines=find_system(p,'FindAll','on','SearchDepth',1,'Type','line');for h=lines(:)',if get_param(h,'SrcPortHandle')==-1 || all(get_param(h,'DstPortHandle')==-1),delete_line(h);end;end
save_system(m,fullfile(root,'models',[m '.slx']));open_system(m);
% Independent reference model with algorithms stored inside MATLAB Function blocks.
f='g1_inprocess_reference';open_system(fullfile(root,'models/g1_reference_frontend.slx'));save_system('g1_reference_frontend',fullfile(root,'models',[f '.slx']));
open_system(fullfile(root,'models',[f '.slx']));
charts=sfroot().find('-isa','Stateflow.Chart');
for k=1:numel(charts)
 c=charts(k);if ~startsWith(c.Path,[f '/'])||isa(c,'Stateflow.EMChart'),continue;end
 st=c.find('-isa','Stateflow.State');if numel(st)~=1,continue;end
 text=st.LabelString;tok=regexp(text,'(\w+)\s*=\s*(task5a_\w+)\(([^;]*)\);','tokens','once');if isempty(tok),continue;end
 path=c.Path;data=c.find('-isa','Stateflow.Data');spec=struct('Name',{},'Scope',{},'DataType',{});
 for j=1:numel(data),spec(j)=struct('Name',data(j).Name,'Scope',data(j).Scope,'DataType',data(j).DataType);end
 pos=get_param(path,'Position');parent=get_param(path,'Parent');name=get_param(path,'Name');ph=get_param(path,'PortHandles');
 from=arrayfun(@(h)get_param(get_param(h,'Line'),'SrcPortHandle'),ph.Inport);
 dest=cell(1,numel(ph.Outport));for j=1:numel(ph.Outport),dest{j}=get_param(get_param(ph.Outport(j),'Line'),'DstPortHandle');end
 delete_block(path);add_block('simulink/User-Defined Functions/MATLAB Function',path,'Position',pos);
 ec=sfroot().find('-isa','Stateflow.EMChart','Path',path);args=tok{3};inputs=strtrim(strsplit(args,','));
 script=sprintf('function %s=compute(%s)\n%%#codegen\n%s=%s(%s);\nend\n',tok{1},args,tok{1},tok{2},args);
 funcs={tok{2}};done={};
 while ~isempty(funcs)
  fun=funcs{1};funcs(1)=[];if any(strcmp(done,fun)),continue;end;done{end+1}=fun;
  body=fileread(fullfile(root,'matlab',[fun '.m']));script=[script newline body]; %#ok<AGROW>
  dep=regexp(body,'\b(task5a_\w+)\s*\(','tokens');for j=1:numel(dep),if ~strcmp(dep{j}{1},fun)&&~any(strcmp(done,dep{j}{1})),funcs{end+1}=dep{j}{1};end;end
 end
 ec.Script=script;
 for j=1:numel(spec)
  d=ec.find('-isa','Stateflow.Data','Name',spec(j).Name);if ~isempty(d),d.DataType=spec(j).DataType;end
 end
 nph=get_param(path,'PortHandles');for j=1:numel(from),lh=get_param(nph.Inport(j),'Line');if lh~=-1,delete_line(lh);end;add_line(parent,from(j),nph.Inport(j),'autorouting','on');end
 for j=1:numel(dest),for h=dest{j}(:)',lh=get_param(h,'Line');if lh~=-1,delete_line(lh);end;add_line(parent,nph.Outport(j),h,'autorouting','on');end;end
end
o=find_system(f,'SearchDepth',1,'BlockType','Outport');
for k=1:numel(o),set_param(o{k},'BusOutputAsStruct','on');end
save_system(f);
fprintf('Built %s with unchanged plant, direct reference signals and dataflow actions.\n',m);
 function copyblock(sid,name,pos)
  source=Simulink.ID.getFullName(sprintf('%s:%d',src,sid));add_block(source,[m '/' name],'Position',pos);
 end
 function wire(source,dest),add_line(m,source,dest,'autorouting','on');end
end
