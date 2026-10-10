function x=nominalState(raw,currentYaw,phase,jointIndices,armActive) %#codegen
% Raw target58, yaw/phase scalar double, four uint32 indices, logical active.
% Indices are original shoulder_y left/right, elbow_y left/right in q23.
coder.cinclude('math.h');x=raw;
if armActive
 if coder.target('MATLAB'),c=cos(currentYaw);s=sin(currentYaw);wave=sin(2*pi*(phase-.15));
 else,c=0;s=0;wave=0;c=coder.ceval('cos',currentYaw);s=coder.ceval('sin',currentYaw);wave=coder.ceval('sin',2*pi*(phase-.15));end
 localVelocity=c*raw(30)+s*raw(31);factor=wave*localVelocity;
 for k=1:4
  j=double(jointIndices(k))+7;
  if mod(k,2)==1,x(j)=x(j)+(-.15)*factor;else,x(j)=x(j)+.15*factor;end
 end
end
end
