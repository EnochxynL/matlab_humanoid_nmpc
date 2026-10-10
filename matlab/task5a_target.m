function out=task5a_target(c,s)
% Preserve accepted secondary 0.8 filter, current yaw anchor, and buffer lag.
coder.cinclude('math.h');
persistent previous target initialized
if isempty(initialized)
 previous=zeros(4,1);target=emptyTarget();initialized=false;
end
if c.reset
 previous=zeros(4,1);target=emptyTarget();target.active_count=uint32(1);
 target.active_time(1)=c.time;x=s.x;x(30:58)=0;x(5:6)=0;target.active_state(:,1)=x;initialized=true;
end
if c.request
 if target.active_count~=uint32(1) || ~c.reset
  target.active_count=uint32(3);target.active_time=target.generated_time;
  target.active_state=target.generated_state;target.active_input=target.generated_input;
 end
 previous=previous*0.8+c.values*(1-0.8);
 yaw=s.x(4);
% Host simulation uses the same standard libm elementary trigonometric
% primitives as the accepted C++ path; all target/filter math stays here.
% This avoids MATLAB host trig versus std::cos one-ULP identity differences.
if coder.target('MATLAB')
 cy=cos(yaw);sy=sin(yaw);
else
 cy=0.0;sy=0.0;
 cy=coder.ceval('cos',yaw);sy=coder.ceval('sin',yaw);
end
v=previous;v(1)=cy*previous(1)-sy*previous(2);v(2)=sy*previous(1)+cy*previous(2);
 pose=s.x(1:6);pose(3)=v(3);pose(5:6)=0;horizon=1.1;middle=0.7*horizon;final=horizon-middle;
 avg=[(s.x(30)+v(1))/2;(s.x(31)+v(2))/2;(s.x(35)+v(4))/2];
 p1=integrate(pose,avg,middle);p2=integrate(p1,[v(1);v(2);v(4)],final);
 q=[-0.05;0;0;0.1;-0.05;0;-0.05;0;0;0.1;-0.05;0;zeros(11,1)];
 vel=[v(1);v(2);0;v(4);0;0;zeros(23,1)];
 target.generated_time=[c.time;c.time+middle;c.time+horizon];
 target.generated_state=[[pose;q;vel],[p1;q;vel],[p2;q;vel]];
end
out=target;
end
function p=integrate(p,v,dt)
p(1)=p(1)+v(1)*dt;p(2)=p(2)+v(2)*dt;p(4)=p(4)+v(3)*dt;p(5:6)=0;
end
function t=emptyTarget()
t=struct('active_count',uint32(0),'active_time',zeros(3,1),'active_state',zeros(58,3),'active_input',zeros(35,3),'generated_time',zeros(3,1),'generated_state',zeros(58,3),'generated_input',zeros(35,3));
end
