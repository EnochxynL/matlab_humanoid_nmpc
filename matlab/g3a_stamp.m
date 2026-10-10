function stamp=g3a_stamp(wallTime)
%G3A_STAMP Transport wall timestamp; never used as physical state time.
stamp=struct('sec',int32(floor(wallTime)),'nanosec',uint32(round(mod(wallTime,1)*1e9)));
if stamp.nanosec==1000000000
 stamp.sec=stamp.sec+1;stamp.nanosec=uint32(0);
end
end
