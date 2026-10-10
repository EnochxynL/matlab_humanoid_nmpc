function g1b_define_buses()
%G1B_DEFINE_BUSES Fixed-size numerical contract, independent of ROS messages.
cfg = g1b_config();
make('RobotJointAction',cfg.actionFields,repmat({29},1,5), ...
    {'rad','rad/s','N*m/rad','N*m/(rad/s)','N*m'},repmat({'double'},1,5));
make('PlantState',cfg.stateFields,{1,3,4,3,3,29,29}, ...
    {'s','m','1','m/s','rad/s','rad','rad/s'},repmat({'double'},1,7));
make('DriveDiagnostics',cfg.driveFields,repmat({29},1,7), ...
    {'N*m','N*m','N*m','N*m','N*m','N*m','1'}, ...
    {'double','double','double','double','double','double','boolean'});
make('WorldDiagnostics',cfg.worldFields,{3,[3 3],3,3}, ...
    {'m','1','rad/s','m/s'},repmat({'double'},1,4));
end
function make(name,names,dimensions,units,types)
elements = repmat(Simulink.BusElement,1,numel(names));
for k=1:numel(names)
    elements(k).Name=names{k};
    elements(k).Dimensions=dimensions{k};
    elements(k).DataType=types{k};
    elements(k).Unit=units{k};
    elements(k).DimensionsMode='Fixed';
end
bus = Simulink.Bus;
bus.Elements=elements;
assignin('base',name,bus);
end
