function out=task5a_swing(schedule)
% Authoritative semantic inputs to the unchanged MPC-side spline consumer.
mode=schedule.current_mode;
out=struct('parameters',[.05;-0.0;.08;-.001;.4;-.15;.3;.005], ...
 'terrain_height',0.0,'left_support',mode==uint8(2)||mode==uint8(3), ...
 'right_support',mode==uint8(1)||mode==uint8(3));
end
