function logs=g3a_log_dataset(out)
logs=g1c_log_dataset(out);signal=Simulink.SimulationData.Signal;
signal.Name='TransportDiagnostics';signal.Values=out.yout{6}.Values;
% Transport updates at 500 Hz; record its held values on the 2 kHz log clock.
clock=logs.get('PlantState').Values.sim_time.Time;
fields=fieldnames(signal.Values);
for k=1:numel(fields)
 ts=setinterpmethod(signal.Values.(fields{k}),'zoh');
 signal.Values.(fields{k})=resample(ts,clock);
end
logs=logs.addElement(signal,'TransportDiagnostics');
end
