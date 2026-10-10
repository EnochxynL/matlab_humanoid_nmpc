function values = g1_series_matrix(signal)
%G1_SERIES_MATRIX Return samples-by-elements for vector or matrix signals.
count = numel(signal.Time);
if signal.IsTimeFirst
    values = reshape(signal.Data,count,[]);
else
    values = reshape(signal.Data,[],count)';
end
end
