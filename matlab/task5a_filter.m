function c=task5a_filter(u)
% Accepted Task3D five-Hz recurrence, exactly once per request.
persistent y lastTime
if isempty(y),y=zeros(4,1);lastTime=0;end
if u.reset,y=u.values;lastTime=u.time;end
if u.request
 dt=u.time-lastTime;assert(dt>=0);
 if dt~=0
  alpha=dt/(dt+1/(2*pi*5));y=alpha*u.values+(1-alpha)*y;lastTime=u.time;
 end
end
c=u;c.values=y;
end
