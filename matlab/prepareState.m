function state=prepareState(position,quaternion,q,linearLocal,angularLocal,qd) %#codegen
% Fixed real double inputs:3,4(wxyz),23,3,3,23 column vectors.
coder.cinclude('math.h');w=quaternion(1);a=quaternion(2);b=quaternion(3);z=quaternion(4);
if coder.target('MATLAB')
 yaw=atan2(2*(w*z+a*b),1-2*(b*b+z*z));pitch=asin(2*(w*b-z*a));roll=atan2(2*(w*a+b*z),1-2*(a*a+b*b));
 sy=sin(pitch);cy=cos(pitch);sx=sin(roll);cx=cos(roll);
else
 yaw=0;pitch=0;roll=0;sy=0;cy=0;sx=0;cx=0;
 yaw=coder.ceval('atan2',2*(w*z+a*b),1-2*(b*b+z*z));pitch=coder.ceval('asin',2*(w*b-z*a));roll=coder.ceval('atan2',2*(w*a+b*z),1-2*(a*a+b*b));
 sy=coder.ceval('sin',pitch);cy=coder.ceval('cos',pitch);sx=coder.ceval('sin',roll);cx=coder.ceval('cos',roll);
end
v=linearLocal;qvec=[a;b;z];uv=cross3(qvec,v);uv=uv+uv;
world=v+w*uv+cross3(qvec,uv);
omega=angularLocal;tmp=sx*omega(2)/cy+cx*omega(3)/cy;
rates=[tmp;cx*omega(2)-sx*omega(3);omega(1)+sy*tmp];
state=[position;yaw;pitch;roll;q;world;rates;qd];
end
function c=cross3(a,b)
c=[a(2)*b(3)-a(3)*b(2);a(3)*b(1)-a(1)*b(3);a(1)*b(2)-a(2)*b(1)];
end
