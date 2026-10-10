function token=task5a_publish(packet)
% One coherent local semantic IPC packet, atomic rename before PlantState send.
% Transport only: no filtering, target/gait math, or physical observation data.
global task5a_runtime
assert(~isempty(task5a_runtime));token=uint64(0);
if ~packet.command.request,return;end
if task5a_runtime.lastPublished==packet.command.tick,return;end
assert(packet.command.session==task5a_session());
task5a_runtime.sequence=task5a_runtime.sequence+uint64(1);
folder=task5a_runtime.folder;temp=fullfile(folder,'reference_packet.tmp');file=fullfile(folder,'reference_packet.bin');
f=fopen(temp,'w','ieee-le');assert(f>=0);
magic=uint64(1413567285);n=double(packet.target.active_count);m=double(packet.schedule.event_count);
fwrite(f,[magic uint64(1) packet.command.session packet.command.epoch task5a_runtime.sequence packet.command.tick uint64(n) uint64(m)],'uint64');
fwrite(f,packet.target.active_time(1:n),'double');fwrite(f,packet.target.active_state(:,1:n),'double');fwrite(f,packet.target.active_input(:,1:n),'double');
fwrite(f,packet.schedule.event_times(1:m),'double');fwrite(f,uint64(packet.schedule.modes(1:m+1)),'uint64');
fwrite(f,packet.schedule.terminal_time,'double');fwrite(f,packet.swing.parameters,'double');fwrite(f,packet.command.values,'double');fwrite(f,magic,'uint64');
assert(fclose(f)==0);[ok,message]=movefile(temp,file,'f');assert(ok,message);
task5a_runtime.lastPublished=packet.command.tick;
task5a_runtime.packets{end+1}=packet;
token=packet.command.tick;
end
