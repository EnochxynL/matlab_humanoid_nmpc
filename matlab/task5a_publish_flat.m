function token=task5a_publish_flat(u)
% Flat transport envelope only; typed semantic bus remains the public boundary.
% Avoid compiling nested-bus-to-MATLAB extrinsic conversion in Stateflow.
assert(numel(u)==428);tick=uint64(u(1));assert(double(tick)==u(1));request=logical(u(2));
if ~request,token=0;return;end
packet.command=struct('tick',tick,'session',task5a_session(),'epoch',uint64(0),'request',request,'values',u(425:428));
packet.target=struct('active_count',uint32(u(3)),'active_time',u(4:6), ...
 'active_state',reshape(u(7:180),58,3),'active_input',reshape(u(181:285),35,3));
packet.schedule=struct('event_count',uint32(u(286)),'event_times',u(287:350), ...
 'modes',uint8(u(351:415)),'terminal_time',u(416));
packet.swing=struct('parameters',u(417:424));token=double(task5a_publish(packet));
end
