function logs = g1b_log_dataset(out)
%G1B_LOG_DATASET Named structured records on a common 2000 Hz snapshot grid.
logs = Simulink.SimulationData.Dataset;
names={'PlantState','RobotJointAction','DriveDiagnostics','WorldDiagnostics'};
for k=1:4
    signal=Simulink.SimulationData.Signal;
    signal.Name=names{k};
    signal.Values=out.yout{k}.Values;
    logs=logs.addElement(signal,names{k});
end
end
