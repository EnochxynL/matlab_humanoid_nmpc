function [active,value,a]=frictionCone(input,contactIndex,stanceFlags,enabled,config) %#codegen
% Original FrictionForceConeConstraint with fixed identity world->terrain.
% config double4x1: friction coefficient, regularization, gripper force,
% Hessian diagonal shift (original default1e-6, NOT a tuning change).
active=enabled && stanceFlags(double(contactIndex)+1);
first=6*double(contactIndex);fx=input(first+1);fy=input(first+2);fz=input(first+3);
fx2=fx*fx;fy2=fy*fy;s=fx2+fy2+config(2);n=sqrt(s);n3=n*s;
value=config(1)*(fz+config(3))-n;
a.dfdx=zeros(1,58);a.dfdu=zeros(1,35);
a.dfdxx=zeros(58,58);a.dfduu=zeros(35,35);a.dfdux=zeros(35,58);
a.dfdu(first+1)=-fx/n;a.dfdu(first+2)=-fy/n;a.dfdu(first+3)=config(1);
a.dfduu(first+1,first+1)=-(fy2+config(2))/n3;
a.dfduu(first+2,first+2)=-(fx2+config(2))/n3;
a.dfduu(first+1,first+2)=fx*fy/n3;
a.dfduu(first+2,first+1)=a.dfduu(first+1,first+2);
for j=1:35,a.dfduu(j,j)=a.dfduu(j,j)-config(4);end
for j=1:58,a.dfdxx(j,j)=a.dfdxx(j,j)-config(4);end
end
