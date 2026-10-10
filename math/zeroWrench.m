function [active,value,dfdx,dfdu]=zeroWrench(input,contactIndex,stanceFlags,enabled) %#codegen
% MATLAB-authored original ZeroWrenchConstraint mathematics.
% input: real double35x1; contactIndex: uint8 scalar 0/1;
% stanceFlags: logical2x1 from ORIGINAL reference manager at the exact time.
% enabled: logical scalar, original setActive state.
% Value/linear approximation remain defined when inactive (OCS2 gates calls).
active=enabled && ~stanceFlags(double(contactIndex)+1);
value=zeros(6,1);dfdx=zeros(6,58);dfdu=zeros(6,35);
first=6*double(contactIndex);
for j=1:6
 value(j)=input(first+j);
 dfdu(j,first+j)=1;
end
end
