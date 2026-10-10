function cfg=g3a_config()
%G3A_CONFIG Transport configuration around the accepted contact plant.
cfg=g1c_config();cfg.modelName='g1_ros2_plant';
cfg.modelFile=fullfile(cfg.projectRoot,'models',[cfg.modelName '.slx']);
cfg.domainId=31;cfg.replyTimeout=0.25;cfg.initialTimeout=3;
cfg.transportFields={'lifecycle','plant_sequence','action_sequence','state_sequence', ...
 'sim_time','send_wall_time','receive_wall_time','latency_s','message_age_s', ...
 'dropped_sequences','out_of_order','stale_actions','timeouts','endpoint_restarts', ...
 'accepted_actions','rejected_actions','coherent_action','wait_s'};
end
