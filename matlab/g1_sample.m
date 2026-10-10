function value = g1_sample(signal,index)
%G1_SAMPLE Extract a sample without assuming timeseries time dimension.
value = getdatasamples(signal,index);
end
