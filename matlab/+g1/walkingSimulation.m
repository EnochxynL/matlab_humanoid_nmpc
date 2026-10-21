function in = walkingSimulation(cfg, formulation, duration, standing)
%WALKINGSIMULATION 配置共用物理模型，替换参考与规划模块，不保存模型变更。
% 两种 planner 的第 4 个输出均为 RobotJointAction，总线各字段为 29 维。
switch formulation
    case 'whole_body'
        planner='g1_planner';reference='g1_reference';
    case 'centroidal'
        planner='g1_centroidal_planner';reference='g1_centroidal_reference';
    otherwise
        error('G1:WalkingConfiguration','Unknown MPC formulation');
end
model='g1_walking';
in=Simulink.SimulationInput(model);
in=in.setBlockParameter([model '/Planner'],'ModelName',planner);
in=in.setBlockParameter([model '/ReferenceGait'],'ModelName',reference);
in=in.setModelParameter('StopTime',num2str(duration,17),'MaxStep','0.0005', ...
    'OutputOption','SpecifiedOutputTimes','OutputTimes',mat2str(0:cfg.driveTs:duration,17), ...
    'SimMechanicsOpenEditorOnUpdate','on','CaptureErrors','on');
names={'g1_q0','g1_qd0','g1_base_p0','g1_base_rpy0','g1_gravity','g1_standing'};
values={cfg.initialQ,cfg.initialQd,cfg.initialPosition,cfg.initialRpy,cfg.gravity,double(standing)};
for k=1:numel(names),in=in.setVariable(names{k},values{k},'Workspace',model);end
end
