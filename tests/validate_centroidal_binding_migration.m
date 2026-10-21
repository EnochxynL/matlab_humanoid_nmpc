function result=validate_centroidal_binding_migration()
[states,packets,config]=binding_replay_inputs('centroidal');
root=fileparts(fileparts(mfilename('fullpath')));
accepted=load(fullfile(root,'runtime','evidence','ocs2_binding','centroidal','replay.mat'),'old');
folder=fullfile(root,'runtime','evidence','binding_migration','centroidal');
config.logDirectory=folder;config.diagnosticsEnabled=true;
g1.plannerMex('init',config);cleanup=onCleanup(@()g1.plannerMex('close'));
n=size(states,1);actual=zeros(n,72);requests=unique(max(0,ceil(((0:384)/80-1e-12)/.002)));
for k=1:n
    tick=k-1;t=tick*.002;request=ismember(tick,requests);
    packet=packets(k,:)';packet(1)=tick;packet(2)=double(request);
    g1.plannerMex('install_reference',packet);
    token=g1.plannerMex('install_observation',states(k,:)',[tick;t;t]);
    token=g1.plannerMex('update_mpc',token,double(request));
    actual(k,:)=g1.plannerMex('query_policy',token)';
    assert(isequal(actual(k,:),g1.plannerMex('query_policy',token)'));
    % Query-only callers drain explicitly; no inverse-dynamics stage follows.
    if request,g1.plannerMex('flush');end
end
clear cleanup;
solves=readtable(fullfile(folder,'solver_timing.csv'));
result=struct('passed',isequal(actual,accepted.old),'ticks',n,'updates',height(solves), ...
    'max_absolute_error',max(abs(actual-accepted.old),[],'all'));
assert(result.passed && result.updates==385);
g1log.writeJson(fullfile(folder,'acceptance.json'),result);
end
