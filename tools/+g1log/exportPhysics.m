function exportPhysics(logs,folder)
%G3C_EXPORT_PHYSICS Common numeric schema, preserving native MAT Dataset.
cfg=g1.loadConfiguration();state=logs.get('PlantState').Values;
T=table(state.sim_time.Time,'VariableNames',{'sim_time'});
fields={'base_position_world','base_orientation_wxyz','base_linear_velocity_local','base_angular_velocity_local'};
labels={{'p_x','p_y','p_z'},{'q_w','q_x','q_y','q_z'},{'v_x','v_y','v_z'},{'w_x','w_y','w_z'}};
for k=1:numel(fields)
 values=g1log.signalMatrix(state.(fields{k}));
 T=[T,array2table(values,'VariableNames',labels{k})]; %#ok<AGROW>
end
q=g1log.signalMatrix(state.q);qd=g1log.signalMatrix(state.qd);
drive=logs.get('DriveDiagnostics').Values;
raw=g1log.signalMatrix(drive.tau_raw);applied=g1log.signalMatrix(drive.tau_applied);
sat=g1log.signalMatrix(drive.saturation_flag);
for j=1:29
 name=cfg.jointNames{j};T.([name '_q'])=q(:,j);T.([name '_qd'])=qd(:,j);
 T.([name '_tau_raw'])=raw(:,j);T.([name '_tau_applied'])=applied(:,j);T.([name '_saturated'])=sat(:,j);
end
contact=logs.get('ContactDiagnostics').Values;
for k=1:2
 names={'Left','Right'};prefix={'left','right'};C=contact.(names{k});
 force=g1log.signalMatrix(C.total_force_world);moment=g1log.signalMatrix(C.total_torque_world);
 T.([prefix{k} '_Fz'])=force(:,3);T.([prefix{k} '_contact'])=g1log.signalMatrix(C.in_contact);
 T.([prefix{k} '_penetration'])=g1log.signalMatrix(C.penetration);
 for j=1:6
  wrench=[force,moment];T.(sprintf('foot%d_wrench_world_%d',k-1,j-1))=wrench(:,j);
 end
end
g1log.writeCsv(T,fullfile(folder,'native_physics.csv'));
T(:,contains(T.Properties.VariableNames,{'tau_','saturated','foot','Fz','contact','penetration'}))=[];
g1log.writeCsv(T,fullfile(folder,'controller_state_native.csv'));
held=logs.get('RobotJointAction').Values;
A=table(held.q_des.Time,'VariableNames',{'sim_time'});
for k=1:5
 values=g1log.signalMatrix(held.(cfg.actionFields{k}));
 for j=1:29,A.([cfg.jointNames{j} '_' cfg.actionFields{k}])=values(:,j);end
end
g1log.writeCsv(A,fullfile(folder,'held_action.csv'));
end
