function [b,Ax,Av,Aa] = swingConfiguration(z,gains,positionEnabled)
%#codegen
% z=[original planner z,zDot,zDDot]; gains=[position,velocity,acceleration].
% Original source uses this exact subtraction order, not a dot-product rewrite.
b = -gains(2)*z(2);
b = b-gains(3)*z(3);
Ax = zeros(1,3);
Av = [0,0,gains(2)];
Aa = [0,0,gains(3)];
if positionEnabled
    b = b-gains(1)*z(1);
    Ax(3) = gains(1);
end
end
