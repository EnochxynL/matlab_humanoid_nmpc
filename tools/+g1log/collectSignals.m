function logs = collectSignals(out)
%COLLECTSIGNALS Preserve the five accepted structured outputs and their order.
logs = Simulink.SimulationData.Dataset;
names = {'PlantState','RobotJointAction','DriveDiagnostics', ...
    'WorldDiagnostics','ContactDiagnostics'};
for k = 1:numel(names)
    signal = Simulink.SimulationData.Signal;
    signal.Name = names{k};
    signal.Values = out.yout{k}.Values;
    logs = logs.addElement(signal,names{k});
end
end
