function v=task5b_read_snapshot(~)
% Read-only; upstream observer atomically renames after complete write.
global task5a_runtime
v=zeros(218,1);
if isempty(task5a_runtime),return;end
file=fullfile(task5a_runtime.folder,'structural_snapshot.bin');
if ~isfile(file),return;end
fid=fopen(file,'rb');assert(fid>=0);cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
v=fread(fid,[218 1],'double');assert(numel(v)==218 && all(isfinite(v)));
end
