function result=task5a_slower(v,x,g)
cfg=task5a_gait_config(g);
result=(abs(v(1))<cfg(1)&&abs(v(2))<cfg(1)&&abs(v(4))<cfg(3)) && (abs(x(30))<cfg(1)+cfg(5)&&abs(x(31))<cfg(1)+cfg(5)&&abs(v(4))<cfg(3)+cfg(6));
end
