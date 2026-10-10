function [value,first,second]=relaxedBarrier(margin,config) %#codegen
% Original RelaxedBarrierPenalty; time unused. Positive delta, mu>=0.
h=margin;mu=config(1);delta=config(2);
if h>delta
 value=-mu*log(h);first=-mu/h;second=mu/(h*h);
else
 deltaH=(h-2.0*delta)/delta;
 value=mu*(-log(delta)+0.5*deltaH*deltaH-0.5);
 first=mu*((h-2.0*delta)/(delta*delta));second=mu/(delta*delta);
end
end
