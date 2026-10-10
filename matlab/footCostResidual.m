function out=footCostResidual(geometry,parameters)
%#codegen
% Prototype for U18. geometry: [unused3;planeError3;v3;omega3;a3;alpha3].
% Parameters retain original 18 reference, 18 sqrt-weight, impact-scaler layout.
% No position tracking term is invented; the original first three errors are zero.
out=zeros(18,1);
for k=1:18
    if k<=3,errorValue=0;
    elseif k<=6,errorValue=geometry(k);
    else,errorValue=geometry(k)-parameters(k);
    end
    out(k)=(errorValue*parameters(k+18))*parameters(37);
end
end
