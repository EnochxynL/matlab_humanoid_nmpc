function writeJson(path,data)
%G1_WRITE_JSON Save machine-readable evidence with actual newlines.
fid = fopen(path,'w');
assert(fid >= 0,'Cannot write %s.',path);
cleanup = onCleanup(@() fclose(fid));
fprintf(fid,'%s\n',jsonencode(data,PrettyPrint=true));
end
