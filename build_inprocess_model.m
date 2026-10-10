function build_inprocess_model()
% Reproducible model construction; original plant/JointDrive remain intact.
root=fileparts(mfilename('fullpath'));cd(root);addpath(fullfile(root,'matlab'));addpath(fullfile(root,'models'));
g1c_define_buses();task5a_define_buses();
m='g1_inprocess_walking';assert(~bdIsLoaded(m),'Close the model before rebuilding it.');
% The checked-in refactored model is the plant/drive template. No old ROS model.
src='g1_inprocess_template';folder=fullfile(root,'build','model_template');
if ~isfolder(folder),mkdir(folder);end
copyfile(fullfile(root,'models',[m '.slx']),fullfile(folder,[src '.slx']));
load_system(fullfile(folder,[src '.slx']));
templateCleanup=onCleanup(@()close_system(src,0)); %#ok<NASGU>
new_system(m);cs=copy(getActiveConfigSet(src));attachConfigSet(m,cs,true);setActiveConfigSet(m,cs.Name);
set_param(m,'InitFcn','g1c_define_buses;task5a_define_buses;', ...
 'StartFcn','','StopFcn','','PreLoadFcn','','PostLoadFcn','','CloseFcn','', ...
 'SimMechanicsOpenEditorOnUpdate','on');
w=get_param(src,'ModelWorkspace');nw=get_param(m,'ModelWorkspace');vars=w.whos;
for k=1:numel(vars),nw.assignin(vars(k).name,w.getVariable(vars(k).name));end
nw.assignin('task5a_standing',0);
cfg=g1b_config();M=zeros(29,23);for k=1:23,M(cfg.mpcIndices(k),k)=1;end
nw.assignin('g1_mpc_to_plant',M);kp=100*ones(29,1);kd=ones(29,1);kp(cfg.mpcIndices)=1200;kd(cfg.mpcIndices)=10;nw.assignin('g1_kp',kp);nw.assignin('g1_kd',kd);
copyblock('PhysicalPlant','PhysicalPlant',[1060 220 1220 360]);copyblock('JointDrive','JointDrive',[825 215 975 345]);
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

copyblock('ReferenceGait','ReferenceGait',[360 70 550 150]);copyblock('ReferencePacket','ReferencePacket',[360 175 550 245]);
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
% ReferencePacket and reference model already contain their embedded algorithms.
% They are authoritative source models; no ROS publication. The statecharts retain four MATLAB math helpers.
save_system(m,fullfile(root,'models',[m '.slx']));open_system(m);
fprintf('Built %s with unchanged plant, direct reference signals and dataflow actions.\n',m);
 function copyblock(block,name,pos)
  source=[src '/' block];add_block(source,[m '/' name],'Position',pos);
 end
 function wire(source,dest),add_line(m,source,dest,'autorouting','on');end
end
