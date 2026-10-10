function approximation=quadraticRunning(x,u,nominalX,nominalU,q,r) %#codegen
% Frozen original WB P is empty. Preserve .5 convention and all derivatives.
dx=x-nominalX;du=u-nominalU;
approximation=struct('f',0,'dfdx',zeros(58,1),'dfdu',zeros(35,1), ...
 'dfdxx',q,'dfduu',r,'dfdux',zeros(35,58));
coder.cstructname(approximation,'Task5CQuadraticRunning');
approximation.dfdx=q*dx;approximation.dfdu=r*du;
approximation.f=0.5*(dx'*approximation.dfdx)+0.5*(du'*approximation.dfdu);
end
