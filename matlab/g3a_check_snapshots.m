function result=g3a_check_snapshots(logs,events)
%G3A_CHECK_SNAPSHOTS Wire publication must match the actual logged plant tick.
state=logs.get('PlantState').Values;clock=state.sim_time.Time;
leaves=0;
for k=1:logs.numElements,leaves=leaves+g1c_check_log_clock(logs.getElement(k).Values,clock);end
wireFields={'sim_time','base_position_world','base_orientation_wxyz', ...
 'base_linear_velocity_local','base_angular_velocity_local','joint_position','joint_velocity'};
plantFields={'sim_time','base_position_world','base_orientation_wxyz', ...
 'base_linear_velocity_local','base_angular_velocity_local','q','qd'};
contact=logs.get('ContactDiagnostics').Values;error=0;count=0;
for k=1:numel(events)
 e=events{k};if ~strcmp(e.event,'accepted'),continue;end
 row=find(abs(clock-e.sim_time)<1e-12,1);assert(~isempty(row));count=count+1;
 for j=1:numel(wireFields)
  expected=g1_sample(state.(plantFields{j}),row);actual=e.state.(wireFields{j});
  delta=max(abs(expected(:)-actual(:)));error=max(error,delta);assert(delta<1e-12);
 end
 expected=[logical(g1_sample(contact.Left.in_contact,row));logical(g1_sample(contact.Right.in_contact,row))];
 assert(isequal(expected,logical(e.state.measured_contact_flags(:))));
end
result=struct('passed',true,'bus_leaves',leaves,'snapshots',count,'max_wire_vs_plant_error',error);
end
