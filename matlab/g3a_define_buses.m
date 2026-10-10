function g3a_define_buses()
g1c_define_buses();cfg=g3a_config();
e=repmat(Simulink.BusElement,1,numel(cfg.transportFields));
for k=1:numel(e),e(k).Name=cfg.transportFields{k};e(k).DataType='double';end
bus=Simulink.Bus;bus.Elements=e;assignin('base','TransportDiagnostics',bus);
end
