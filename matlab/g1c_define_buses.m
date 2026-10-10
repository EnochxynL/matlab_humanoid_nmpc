function g1c_define_buses()
g1b_define_buses(); cfg=g1c_config();
e=repmat(Simulink.BusElement,1,numel(cfg.footFields));
for k=1:numel(e)
 e(k).Name=cfg.footFields{k}; e(k).Dimensions=cfg.footDims(k);
 e(k).DataType='double'; e(k).DimensionsMode='Fixed';
 units={'1','N','N','N','N*m','m','1','m','1','N','m'}; e(k).Unit=units{k};
end
e(1).DataType='boolean'; e(9).DataType='boolean';
b=Simulink.Bus; b.Elements=e; assignin('base','FootContactDiagnostics',b);
e=repmat(Simulink.BusElement,1,2);
e(1).Name='Left'; e(2).Name='Right';
for k=1:2, e(k).DataType='Bus: FootContactDiagnostics'; end
b=Simulink.Bus; b.Elements=e; assignin('base','ContactDiagnostics',b);
end
