function task5a_export_semantics(p,file)
f=fopen(file,'w');assert(f>=0);cleanup=onCleanup(@()fclose(f));fprintf(f,'tick,time,kind,knot,index,value\n');
for row=1:numel(p.command.time.Time)
 if ~p.command.request.Data(row),continue;end
 tick=p.command.tick.Data(row);t=p.command.time.Data(row);
 emit('filtered',0,p.command.values.Data(row,:));
 count=double(p.target.active_count.Data(row));
 activeTimes=p.target.active_time.Data(row,:);generatedTimes=p.target.generated_time.Data(row,:);
 % Multidimensional timeseries Data uses [58,3,time] (time last).
 activeState=squeeze(p.target.active_state.Data(:,:,row));generatedState=squeeze(p.target.generated_state.Data(:,:,row));
 activeInput=squeeze(p.target.active_input.Data(:,:,row));generatedInput=squeeze(p.target.generated_input.Data(:,:,row));
 for k=1:count,emit('active_time',k-1,activeTimes(k));emit('active_state',k-1,activeState(:,k));emit('active_input',k-1,activeInput(:,k));end
 for k=1:3,emit('generated_time',k-1,generatedTimes(k));emit('generated_state',k-1,generatedState(:,k));emit('generated_input',k-1,generatedInput(:,k));end
 n=double(p.schedule.event_count.Data(row));emit('event',0,p.schedule.event_times.Data(row,1:n));emit('mode',0,p.schedule.modes.Data(row,1:n+1));
 emit('adapter_terminal',0,p.schedule.terminal_time.Data(row));
 emit('swing_parameter',0,p.swing.parameters.Data(row,:));
 emit('current_mode',0,p.schedule.current_mode.Data(row));
 emit('support',0,[p.swing.left_support.Data(row),p.swing.right_support.Data(row)]);
end
 function emit(kind,knot,values)
  for idx=1:numel(values),fprintf(f,'%u,%.17g,%s,%d,%d,%.17g\n',tick,t,kind,knot,idx-1,double(values(idx)));end
 end
end
