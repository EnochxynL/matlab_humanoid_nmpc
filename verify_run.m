function result=verify_run()
% Independently check transport and actions, export actual physical evidence.
root=setup_project();session=jsondecode(fileread(fullfile(root,'runtime/current_session.json')));
folder=session.folder;data=load(fullfile(folder,'plant_and_reference.mat'),'logs','events','counts');
assert(~any(data.counts),'Protocol errors recorded.');
result.snapshots=g3a_check_snapshots(data.logs,data.events);
result.actions=g3b_check_actions(data.logs,data.events,fullfile(folder,'controller_action.csv'));
g3c_export_physics(data.logs,folder);
state=data.logs.get('PlantState').Values;
t=state.sim_time.Time;p=g1_series_matrix(state.base_position_world);
q=g1_series_matrix(state.base_orientation_wxyz);
assert(abs(t(end)-session.duration)<1e-12,'Incomplete physical run.');
fields=fieldnames(state);
for k=1:numel(fields),assert(all(isfinite(state.(fields{k}).Data),'all'),'Nonfinite plant state.');end
pitch=asin(max(-1,min(1,2*(q(:,1).*q(:,3)-q(:,4).*q(:,2)))));
result.duration_s=t(end);result.position_start_m=p(1,:);result.position_end_m=p(end,:);
result.displacement_m=p(end,:)-p(1,:);result.height_range_m=[min(p(:,3)),max(p(:,3))];
result.max_abs_pitch_deg=max(abs(rad2deg(pitch)));
contact=data.logs.get('ContactDiagnostics').Values;
for name={'Left','Right'}
 c=contact.(name{1});fz=g1_series_matrix(c.total_force_world);
 flags=logical(g1_series_matrix(c.in_contact));
 result.contact.(name{1})=struct('transitions',sum(abs(diff(double(flags)))), ...
   'force_z_range_N',[min(fz(:,3)),max(fz(:,3))]);
end
result.protocol_counts=data.counts;result.folder=folder;
g1_write_json(fullfile(folder,'verification.json'),result);disp(result);
end
