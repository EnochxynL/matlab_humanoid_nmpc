function s=task5a_observation(p,session)
% Reproduce ControlTimeWBMpcController::updateMpcState and exact reduced order.
coder.cinclude('math.h');
tick=uint64(round(p.sim_time/.002));q=p.base_orientation_wxyz;
w=q(1);a=q(2);b=q(3);z=q(4);
yawArgY=2*(w*z+a*b);yawArgX=1-2*(b*b+z*z);
pitchArg=2*(w*b-z*a);rollArgY=2*(w*a+b*z);rollArgX=1-2*(a*a+b*b);
if coder.target('MATLAB')
 yaw=atan2(yawArgY,yawArgX);pitch=asin(pitchArg);roll=atan2(rollArgY,rollArgX);
 sy=sin(pitch);cy=cos(pitch);sx=sin(roll);cx=cos(roll);
else
 yaw=0;pitch=0;roll=0;sy=0;cy=0;sx=0;cx=0;
 yaw=coder.ceval('atan2',yawArgY,yawArgX);pitch=coder.ceval('asin',pitchArg);roll=coder.ceval('atan2',rollArgY,rollArgX);
 sy=coder.ceval('sin',pitch);cy=coder.ceval('cos',pitch);sx=coder.ceval('sin',roll);cx=coder.ceval('cos',roll);
end
% Eigen Quaternion::_transformVector expression ordering.
v=p.base_linear_velocity_local;qvec=[a;b;z];uv=cross3(qvec,v);uv=uv+uv;vw=v+w*uv+cross3(qvec,uv);
omega=p.base_angular_velocity_local;tmp=sx*omega(2)/cy+cx*omega(3)/cy;
d=[tmp;cx*omega(2)-sx*omega(3);omega(1)+sy*tmp];
% PlantState uses g1_config().jointNames (alphabetical), not URDF traversal.
% Exact accepted name-to-index lookup g1_config().mpcIndices, frozen here for
% code generation; regression compares every component to the C++ observation.
ids=[4 5 6 7 1 2 17 18 19 20 14 15 29 28 27 8 9 10 3 21 22 23 16];
x=[p.base_position_world;yaw;pitch;roll;p.q(ids);vw;d;p.qd(ids)];
s=struct('tick',tick,'session',session,'epoch',uint64(0),'raw_backend_time',p.sim_time,'x',x);
end
function c=cross3(a,b)
c=[a(2)*b(3)-a(3)*b(2);a(3)*b(1)-a(1)*b(3);a(1)*b(2)-a(2)*b(1)];
end
