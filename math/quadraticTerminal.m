function [value,gradient,hessian]=quadraticTerminal(x,desired,q) %#codegen
% x/desired real double58x1; q real double58x58 (original scaled Q_final).
deviation=x-desired;hessian=q;gradient=q*deviation;
value=0.5*(deviation'*gradient);
end
