function result=task5a_faster(v,x,g)
cfg=task5a_gait_config(g);
result=(abs(v(1))>cfg(2)||abs(v(2))>cfg(2)||abs(v(4))>cfg(4)) && (abs(x(30))>cfg(2)-cfg(5)||abs(x(31))>cfg(2)-cfg(5)||abs(x(33))>cfg(4)-cfg(6));
end
