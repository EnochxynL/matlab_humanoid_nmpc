function in = g1b_sim_input(cfg,data,stopTime,q0,qd0,p0,rpy0,gravity)
%G1B_SIM_INPUT Set action and initial-state fixtures without changing topology.
if nargin<4, q0=cfg.initialQ; end
if nargin<5, qd0=zeros(29,1); end
if nargin<6, p0=cfg.initialPosition; end
if nargin<7, rpy0=cfg.initialRpy; end
if nargin<8, gravity=cfg.gravity; end
assert(isfinite(stopTime)&&stopTime>0);
in=Simulink.SimulationInput(cfg.modelName);
in=in.setModelParameter('StopTime',sprintf('%.17g',stopTime), ...
    'OutputOption','SpecifiedOutputTimes','OutputTimes',mat2str(0:cfg.driveTs:stopTime,17));
for k=1:5
    name=cfg.actionFields{k};
    in=in.setVariable(['g1_action_' name],data.(name),'Workspace',cfg.modelName);
end
names={'g1_q0','g1_qd0','g1_base_p0','g1_base_rpy0','g1_gravity'};
values={q0,qd0,p0,rpy0,gravity};
for k=1:numel(names), in=in.setVariable(names{k},values{k},'Workspace',cfg.modelName); end
end
