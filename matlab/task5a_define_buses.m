function task5a_define_buses()
% Typed semantic interface; fixed capacities fail closed, no silent truncation.
def('Task5AReferenceState', {'tick',1,'uint64';'session',1,'uint64';'epoch',1,'uint64';'raw_backend_time',1,'double';'x',58,'double'});
def('Task5ALocomotionCommand', {'vx',1,'double';'vy',1,'double';'height',1,'double';'yaw_rate',1,'double'});
def('Task5AProcessedCommand', {'tick',1,'uint64';'session',1,'uint64';'epoch',1,'uint64';'time',1,'double';'request',1,'boolean';'reset',1,'boolean';'values',4,'double'});
def('Task5ATarget', {'active_count',1,'uint32';'active_time',3,'double';'active_state',[58 3],'double';'active_input',[35 3],'double';'generated_time',3,'double';'generated_state',[58 3],'double';'generated_input',[35 3],'double'});
def('Task5AGaitSchedule', {'event_count',1,'uint32';'event_times',64,'double';'modes',65,'uint8';'gait_index',1,'uint8';'current_mode',1,'uint8';'terminal_time',1,'double'});
def('Task5ASwingReference', {'parameters',8,'double';'terrain_height',1,'double';'left_support',1,'boolean';'right_support',1,'boolean'});
def('Task5AReferenceGait', {'command',1,'Bus: Task5AProcessedCommand';'target',1,'Bus: Task5ATarget';'schedule',1,'Bus: Task5AGaitSchedule';'swing',1,'Bus: Task5ASwingReference'});
end
function def(name,spec)
b=Simulink.Bus;
for k=1:size(spec,1)
 e=Simulink.BusElement;e.Name=spec{k,1};e.Dimensions=spec{k,2};e.DataType=spec{k,3};b.Elements(k)=e;
end
assignin('base',name,b);
end
