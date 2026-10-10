function c=task5a_command(s,u)
% Canonical physical identity and accepted first-observation-after-60Hz-deadline.
% Repeated physical ticks are read-only; session/epoch changes reset all state.
persistent session epoch slot lastTick initialized
if isempty(initialized)
 initialized=false;session=uint64(0);epoch=uint64(0);slot=uint64(0);lastTick=uint64(0);
end
reset=~initialized || s.session~=session || s.epoch~=epoch;
t=double(s.tick)*0.002;
assert(isfinite(s.raw_backend_time) && abs(s.raw_backend_time-t)<=1e-12);
if reset
 session=s.session;epoch=s.epoch;slot=uint64(0);initialized=true;
else
 assert(s.tick>=lastTick,'Control tick rollback requires a new epoch.');
end
request=reset || (s.tick>lastTick && t+1e-12>=double(slot)/60);
if request,slot=slot+uint64(1);end
lastTick=s.tick;
c=struct('tick',s.tick,'session',session,'epoch',epoch,'time',t,'request',request,'reset',reset,'values',[u.vx;u.vy;u.height;u.yaw_rate]);
end
