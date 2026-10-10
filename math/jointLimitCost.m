function [value,gradient,hessian]=jointLimitCost(q,lower,upper,config) %#codegen
% q/lower/upper real double23x1; config double2x1 [mu;delta].
% The WB model packs actuated q at x(7:29); offset in the original is zero.
up=zeros(23,1);lo=zeros(23,1);gradient=zeros(58,1);hessian=zeros(58,58);
for k=1:23
 [up(k),du,hu]=polynomialBarrier(upper(k)-q(k),config);
 [lo(k),dl,hl]=polynomialBarrier(q(k)-lower(k),config);
 gradient(6+k)=dl-du;hessian(6+k,6+k)=hl+hu;
end
value=sum(up)+sum(lo)+0.0;
end
