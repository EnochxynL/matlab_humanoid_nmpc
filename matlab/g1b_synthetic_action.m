function data = g1b_synthetic_action(cfg,mode,stopTime)
%G1B_SYNTHETIC_ACTION Deterministic 500 Hz action fields; no MPC controller.
arguments
    cfg struct
    mode char = 'zero'
    stopTime (1,1) double {mustBePositive,mustBeFinite} = 0.01
end
time = (0:cfg.actionTs:stopTime+cfg.actionTs)';
fields = cfg.actionFields;
for k=1:numel(fields), values.(fields{k})=zeros(numel(time),29); end
switch mode
    case 'zero'
    case 'feedforward'
        values.tau_ff(:,:) = repmat(0.5*sin((1:29)),numel(time),1);
    case 'p_only'
        values.q_des(:,:) = repmat(cfg.initialQ'+0.01*cos(1:29),numel(time),1);
        values.kp(:,:) = 20;
    case 'd_only'
        values.qd_des(:,:) = repmat(0.03*sin(1:29),numel(time),1);
        values.kd(:,:) = 2;
    case {'combined','step'}
        values.q_des(:,:) = repmat(cfg.initialQ'+0.01*cos(1:29),numel(time),1);
        values.qd_des(:,:) = repmat(0.03*sin(1:29),numel(time),1);
        values.kp(:,:) = repmat(10+0.1*(1:29),numel(time),1);
        values.kd(:,:) = repmat(1+0.01*(1:29),numel(time),1);
        values.tau_ff(:,:) = repmat(0.5*sin(1:29),numel(time),1);
        if strcmp(mode,'step')
            rows=time>=0.004;
            values.q_des(rows,:) = values.q_des(rows,:)+0.008;
            values.qd_des(rows,:) = values.qd_des(rows,:)-0.02;
            values.kp(rows,:) = values.kp(rows,:)+3;
            values.kd(rows,:) = values.kd(rows,:)+0.5;
            values.tau_ff(rows,:) = values.tau_ff(rows,:)+0.2;
        end
    otherwise
        error('G1B:ActionMode','Unknown synthetic action mode: %s',mode);
end
for k=1:numel(fields)
    data.(fields{k}) = timeseries(values.(fields{k}),time);
    data.(fields{k}) = setinterpmethod(data.(fields{k}),'zoh');
end
end
