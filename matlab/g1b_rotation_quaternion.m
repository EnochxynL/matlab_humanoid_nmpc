function q = g1b_rotation_quaternion(R)
%#codegen
% R_WB maps pelvis-local vectors to World; q is [w;x;y;z].
% Branch on the largest diagonal for stability near rotations of pi.
R=reshape(R,3,3);
q=zeros(4,1);
t=trace(R);
if t>0
    s=2*sqrt(1+t);
    q=[s/4;(R(3,2)-R(2,3))/s;(R(1,3)-R(3,1))/s;(R(2,1)-R(1,2))/s];
elseif R(1,1)>=R(2,2) && R(1,1)>=R(3,3)
    s=2*sqrt(1+R(1,1)-R(2,2)-R(3,3));
    q=[(R(3,2)-R(2,3))/s;s/4;(R(1,2)+R(2,1))/s;(R(1,3)+R(3,1))/s];
elseif R(2,2)>=R(3,3)
    s=2*sqrt(1+R(2,2)-R(1,1)-R(3,3));
    q=[(R(1,3)-R(3,1))/s;(R(1,2)+R(2,1))/s;s/4;(R(2,3)+R(3,2))/s];
else
    s=2*sqrt(1+R(3,3)-R(1,1)-R(2,2));
    q=[(R(2,1)-R(1,2))/s;(R(1,3)+R(3,1))/s;(R(2,3)+R(3,2))/s;s/4];
end
q=q/sqrt(sum(q.*q));
if q(1)<0, q=-q; end
end
