function [value,first,second]=polynomialBarrier(margin,config) %#codegen
% Config double2x1 [mu;delta]; real scalar margin. Preserve executed source,
% not the inconsistent formula printed in the upstream header comment.
h=margin;mu=config(1);delta=config(2);
if h<=0
 penalty=0.5*h*h-delta*h/2+delta*delta/6;
 value=penalty*mu;first=(h-delta/2)*mu;second=mu;
elseif h>0 && h<delta
 penalty=-h*h*h/(6*delta)+0.5*h*h-delta*h/2+delta*delta/6;
 value=penalty*mu;first=(-h*h/(2*delta)+h-delta/2)*mu;second=mu*(-h/delta+1);
else
 value=0;first=0;second=0;
end
end
