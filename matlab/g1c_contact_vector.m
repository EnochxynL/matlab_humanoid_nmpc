function y=g1c_contact_vector(u,groundHeight,forceThreshold,copThreshold)
%G1C_CONTACT_VECTOR Aggregate measured base-on-follower contact wrench.
% Inputs: depth[4], Fn[4], friction[3x4], total F[3x4], total M[3x4],
% foot-contact world origin[3], R_WC[3x3]. Torques sensed about plane origin.
depth=u(1:4); fn=u(5:8); friction=reshape(u(9:20),3,4);
f=sum(reshape(u(21:32),3,4),2); mGround=sum(reshape(u(33:44),3,4),2);
p=u(45:47); R=reshape(u(48:56),3,3); ground=[0;0;groundHeight];
m=mGround+cross(ground-p,f); % moment shift BEFORE rotating to local frame
wrench=[R'*f;R'*m]; normal=sum(fn);
valid=normal>copThreshold && f(3)>copThreshold;
cop=[NaN;NaN];
if valid
 % World-plane intersection, then express relative to contact frame.
 worldCop=ground+[-mGround(2)/f(3);mGround(1)/f(3);0];
 localCop=R'*(worldCop-p); cop=localCop(1:2);
end
y=[double(normal>forceThreshold);normal;sum(friction,2);f;m; ...
 max(depth);wrench;cop;double(valid);fn;depth];
end
