function cfg=task5a_gait_config(index)
% Columns: min/max linear, min/max angular, linear/angular error threshold.
configs=[-.1 .1 -.1 .1 10 10;.05 .3 .05 .2 .05 .05;.25 .5 .15 .35 .05 .05;.45 .7 .3 .55 .1 .1;.65 .9 .5 .7 .2 .2;.8 1.3 .65 10 .2 .2;1.2 10 .65 10 .2 .2];
assert(index<=6);cfg=configs(double(index)+1,:);
end
