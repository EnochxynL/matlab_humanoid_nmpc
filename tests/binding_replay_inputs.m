function [states,packets,config]=binding_replay_inputs(formulation)
% Same frozen kinematics and targets for legacy and migrated execution paths.
root=fileparts(fileparts(mfilename('fullpath')));g1.initialize();
golden=load(fullfile(root,'runtime','evidence','observation_hybrid','conversion_values.mat'));
fixture=load(fullfile(root,'runtime','evidence','core_extraction','fixture.mat'));
config=g1.plannerConfiguration(root,'',false,formulation);
states=golden.expected(1:size(fixture.raw,1),:);packets=fixture.ref;
if strcmp(formulation,'whole_body'),return;end
model=ocs2.Problem(config);cleanup=onCleanup(@()delete(model)); %#ok<NASGU>
settings=model.modelSettings();qv=states;raw=packets;
n=size(qv,1);states=zeros(n,35);packets=zeros(n,359);latest=raw(1,:)';
for k=1:n
    states(k,:)=model.convertObservation(qv(k,:)')';
    if raw(k,2)~=0,latest=raw(k,:)';end
    a=latest;target=reshape(a(7:180),58,3);converted=zeros(35,3);
    for j=1:3,converted(:,j)=model.convertObservation(target(:,j));end
    packets(k,:)=[a(1:6);converted(:);a(181:285);a(286:416);settings.swing;a(425:428)]';
end
end
