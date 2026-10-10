function logs=g1c_log_dataset(out)
%G1C_LOG_DATASET Preserve accepted four buses; append diagnostic contact bus.
logs=g1b_log_dataset(out);
signal=Simulink.SimulationData.Signal;
signal.Name='ContactDiagnostics'; signal.Values=out.yout{5}.Values;
logs=logs.addElement(signal,'ContactDiagnostics');
end
