function count=g1c_check_log_clock(value,t)
%G1C_CHECK_LOG_CLOCK Verify every bus leaf uses the common snapshot clock.
count=0;
if isa(value,'timeseries')
 assert(isequal(value.Time,t),'Task2C:Clock','Bus leaf clock differs.');
 count=1;
elseif isstruct(value)
 fields=fieldnames(value);
 for k=1:numel(fields)
  count=count+g1c_check_log_clock(value.(fields{k}),t);
 end
else
 error('Task2C:LogType','Unexpected logged bus value.');
end
end
