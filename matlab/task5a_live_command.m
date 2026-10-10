function c=task5a_live_command(s,standing)
% Accepted Task4A command, evaluated on canonical physical identity.
vx=0;if ~standing && s.tick>=uint64(250),vx=.1;end
c=struct('vx',vx,'vy',0.0,'height',.8,'yaw_rate',0.0);
end
