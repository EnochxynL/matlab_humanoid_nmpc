function out=task5a_schedule(c,gait)
% Port of the accepted raw schedule mutation order. Do not remove duplicate
% STANCE events or rewrite floating expressions at hybrid boundaries.
persistent events modes n template previousGait held
if isempty(n)
 n=uint32(1);events=zeros(64,1);events(1)=0.5;modes=uint8(3)*ones(65,1,'uint8');template=uint8(0);previousGait=uint8(0);held=emptySchedule();
end
if c.reset
 n=uint32(1);events(:)=0;events(1)=.5;modes(:)=uint8(3);template=uint8(0);previousGait=uint8(0);held=emptySchedule();
end
if c.request
 t=c.time;final=t+1.1;h=final-t;
 [events,modes,n]=window(events,modes,n,template,t-h,final+h);
 held.event_count=n;held.event_times=events;held.modes=modes;held.gait_index=gait;
 held.current_mode=modeAt(events,modes,n,t);held.terminal_time=events(double(n))+1;
 if gait~=previousGait
  [events,modes,n]=window(events,modes,n,template,t,final+h);
  earliest=.7*final+.3*t;idx=1;
  while idx<=double(n) && events(idx)<=earliest,idx=idx+1;end
  if idx>double(n)
   next=final;
  elseif modeAt(events,modes,n,events(idx))==uint8(2)
   assert(idx>1);next=events(idx-1);
  else
   next=events(idx);
  end
  template=gait;
  idx=1;while idx<=double(n)&&events(idx)<next,idx=idx+1;end
  n=uint32(idx-1);
  % phaseTransitionStanceTime=0, including transitions from non-STANCE.
  [events,modes,n]=tile(events,modes,n,template,next,1.5*h);
  previousGait=gait;
 end
end
out=held;
end
function [e,m,n]=window(e,m,n,g,lo,hi)
idx=1;while idx<=double(n)&&e(idx)<lo,idx=idx+1;end
if idx>1
 remove=idx-2;keep=double(n)-remove;
 e(1:keep)=e(remove+1:double(n));m(1:keep+1)=m(remove+1:double(n)+1);n=uint32(keep);m(1)=uint8(3);
end
assert(n>0);start=e(double(n));n=n-uint32(1);
[e,m,n]=tile(e,m,n,g,start,hi);
end
function [e,m,n]=tile(e,m,n,g,start,final)
[times,seq,count]=gaitTemplate(g);
if n>0,assert(start>e(double(n)));end
n=n+uint32(1);assert(n<=64);e(double(n))=start;
while e(double(n))<final
 for j=1:count
  m(double(n)+1)=seq(j);delta=times(j+1)-times(j);
  n=n+uint32(1);assert(n<=64);e(double(n))=e(double(n)-1)+delta;
 end
end
m(double(n)+1)=uint8(3);
% Clear unused storage only, preserving every active event and mode.
e(double(n)+1:64)=0;m(double(n)+2:65)=uint8(0);
end
function [t,s,count]=gaitTemplate(g)
t=zeros(5,1);s=zeros(4,1,'uint8');count=1;
if g==0,t(2)=.5;s(1)=uint8(3);
elseif g==1,t=[0;.65;.85;1.5;1.7];s=uint8([2;3;1;3]);count=4;
elseif g==2,t=[0;.6;.7;1.3;1.4];s=uint8([2;3;1;3]);count=4;
elseif g==3,t(1:3)=[0;.6;1.2];s(1:2)=uint8([2;1]);count=2;
elseif g==4,t(1:3)=[0;.55;1.1];s(1:2)=uint8([2;1]);count=2;
elseif g==5,t(1:3)=[0;.5;1];s(1:2)=uint8([2;1]);count=2;
else,t=[0;.3;.4;.7;.8];s=uint8([2;0;1;0]);count=4;
end
end
function mode=modeAt(e,m,n,t)
i=1;while i<=double(n)&&e(i)<t,i=i+1;end;mode=m(i);
end
function s=emptySchedule()
s=struct('event_count',uint32(0),'event_times',zeros(64,1),'modes',zeros(65,1,'uint8'),'gait_index',uint8(0),'current_mode',uint8(3),'terminal_time',0.0);
end
