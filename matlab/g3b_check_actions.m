function result=g3b_check_actions(logs,events,csvFile)
%G3B_CHECK_ACTIONS Compare pre-serialization C++ actions with actual held bus.
opts=detectImportOptions(csvFile,'Delimiter',',');
opts=setvartype(opts,{'session','state_sequence','action_sequence'},'uint64');
opts=setvartype(opts,'joint_name','string');tableData=readtable(csvFile,opts);
cfg=g3a_config();bus=logs.get('RobotJointAction').Values;clock=bus.q_des.Time;
maximum=0;packets=0;wristChecks=0;
for k=1:numel(events)
 event=events{k};if ~strcmp(event.event,'accepted'),continue;end
 rows=tableData.session==event.session_id & tableData.state_sequence==event.plant_sequence;
 packet=tableData(rows,:);assert(height(packet)==29,'Missing/duplicate raw controller action rows.');
 [present,indices]=ismember(cfg.jointNames,packet.joint_name);assert(all(present));packet=packet(indices,:);
 row=find(abs(clock-event.sim_time)<1e-12,1);assert(~isempty(row));packets=packets+1;
 for j=1:5
  field=cfg.actionFields{j};actual=g1_sample(bus.(field),row);expected=packet.(field);
  delta=max(abs(actual(:)-expected(:)));maximum=max(maximum,delta);assert(delta<1e-12);
 end
 wrists=packet(cfg.wristIndices,:);
 assert(all(wrists.q_des==0 & wrists.qd_des==0 & wrists.kp==100 & wrists.kd==1 & wrists.tau_ff==0));
 wristChecks=wristChecks+height(wrists);
end
assert(packets>0,'No real controller action reached Simulink.');
result=struct('passed',true,'packets',packets,'max_action_mapping_error',maximum,'wrist_checks',wristChecks);
end
