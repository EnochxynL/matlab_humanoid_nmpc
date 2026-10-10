function task5b_define_buses()
% Read-only typed boundaries. Packed x/u remain the original C++ API.
def('WBMpcState',{'position_world',3,'double','m, world';'euler_zyx',3,'double','rad, local-to-world yaw pitch roll';'q_mpc',23,'double','rad, accepted MPC joint order';'velocity_world',3,'double','m/s, world';'euler_zyx_dot',3,'double','rad/s, Euler coordinate derivatives';'qd_mpc',23,'double','rad/s, MPC order';'packed',58,'double','mixed units; WBAccelMpcRobotModel'});
def('WBMpcInput',{'left_wrench',6,'double','[N,N,N,Nm,Nm,Nm], LOCAL_WORLD_ALIGNED';'right_wrench',6,'double','same, right foot';'qdd_mpc',23,'double','rad/s2, MPC order';'packed',35,'double','wrench12 then qdd23'});
def('WBMpcObservation',{'control_tick',1,'uint64','physical time identity, 2 ms';'raw_backend_time',1,'double','s, unmodified evidence';'canonical_control_time',1,'double','s, double(tick)*.002';'state',1,'Bus: WBMpcState','mapped RobotState';'observation_input',35,'double','zeros, original controller behavior';'mode',1,'uint8','observed controller STANCE=3'});
def('WBMpcReference',{'target',1,'Bus: Task5ATarget','accepted Simulink target58/input35 knots';'swing',1,'Bus: Task5ASwingReference','accepted parameters/terrain/support'});
def('WBMpcMode',{'schedule',1,'Bus: Task5AGaitSchedule','accepted raw events/mode sequence';'mode',1,'uint8','0 flight,1/2 single support,3 double'});
def('WBMpcPolicySample',{'valid',1,'boolean','false until first actual policy';'control_tick',1,'uint64','derived identity, 2 ms';'observation_time',1,'double','s';'query_time',1,'double','s, observation+.005';'policy_start',1,'double','s';'policy_end',1,'double','s';'mode',1,'uint8','planned mode at query';'state',1,'Bus: WBMpcState','current interpolated desired state, not entire policy';'input',1,'Bus: WBMpcInput','current feedforward input35'});
def('InverseDynamicsCommand',{'valid',1,'boolean','read-only same-source reconstruction';'generalized_acceleration',29,'double','base6 Euler coordinate acceleration then joint23';'left_wrench',6,'double','N/Nm, world-aligned contact frame origin';'right_wrench',6,'double','same';'q_des',23,'double','rad, MPC order';'qd_des',23,'double','rad/s, MPC order';'tau_ff',23,'double','Nm, cloned original computeJointTorques; verified against actual action'});
def('SolverDiagnostics',{'valid',1,'boolean','actual public SqpSolver metrics';'total_iterations',1,'double','iterations since reset, not status';'objective',1,'double','actual integrated cost; barrier costs may be negative';'dynamics_sse',1,'double','original SSE';'equality_sse',1,'double','original SSE';'lq_wall_ms',1,'double','wall ms';'qp_wall_ms',1,'double','wall ms';'linesearch_wall_ms',1,'double','wall ms';'controller_wall_ms',1,'double','wall ms'});
def('PredictionDiagnostics',{'valid',1,'boolean','clone of actual original dynamics';'sample_time',1,'double','s, one interior knot per actual solve';'derivative',58,'double','mixed state derivative units; not current query sample';'A_size',2,'double','[58 58]';'B_size',2,'double','[58 35]'});
def('Task5BExposure',{'policy',1,'Bus: WBMpcPolicySample','read-only policy sample';'inverse_dynamics',1,'Bus: InverseDynamicsCommand','read-only ID';'solver',1,'Bus: SolverDiagnostics','actual solver metrics';'prediction',1,'Bus: PredictionDiagnostics','selected clone sample'});
end
function def(name,spec)
b=Simulink.Bus;b.Description='Task5B READ-ONLY semantic exposure; C++ remains authority.';
for k=1:size(spec,1)
 e=Simulink.BusElement;e.Name=spec{k,1};e.Dimensions=spec{k,2};e.DataType=spec{k,3};e.Description=spec{k,4};b.Elements(k)=e;
end
assignin('base',name,b);
end
