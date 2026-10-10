function writeCsv(T,file)
%G3C_PRECISE_CSV Numeric tables with 17 significant digits for exact replay.
assert(all(varfun(@isnumeric,T,'OutputFormat','uniform') | varfun(@islogical,T,'OutputFormat','uniform')));
fileID=fopen(file,'w');assert(fileID>=0);cleanup=onCleanup(@()fclose(fileID));
fprintf(fileID,'%s\n',strjoin(T.Properties.VariableNames,','));
format=[repmat('%.17g,',1,width(T)-1) '%.17g\n'];
fprintf(fileID,format,double(table2array(T)).');
end
