function cfg=g1c_config()
%G1C_CONFIG Explicit contact engineering parameters, not MuJoCo equivalents.
cfg=g1b_config();
cfg.modelName='g1_contact_plant';
cfg.modelFile=fullfile(cfg.projectRoot,'models',[cfg.modelName '.slx']);
cfg.geometry=jsondecode(fileread(fullfile(cfg.projectRoot,'data','g1_contact_geometry.json')));
cfg.contactPoints=cfg.geometry.feet(1).point_cloud_contact_m;
cfg.groundHeight=0;
cfg.normalStiffness=1e5; cfg.normalDamping=1e3; cfg.transitionWidth=1e-3;
cfg.muStatic=1.5; cfg.muDynamic=1.2; cfg.criticalVelocity=1e-3;
cfg.contactThreshold=1; cfg.copThreshold=5;
cfg.fixtureMass=10; cfg.fixtureHeight=0.02;
cfg.footFields={'in_contact','normal_force','friction_force_world', ...
    'total_force_world','total_torque_world','penetration','wrench_local', ...
    'cop_local','cop_valid','point_normal_force','point_penetration'};
cfg.footDims=[1 1 3 3 3 1 6 2 1 4 4];
end
