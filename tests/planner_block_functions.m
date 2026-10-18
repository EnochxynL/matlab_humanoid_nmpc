function [kernels,cleanup] = planner_block_functions()
%PLANNER_BLOCK_FUNCTIONS Test entry points generated from actual model code.
% The model is the implementation source; temporary .m files exist only for tests.
root=fileparts(fileparts(mfilename('fullpath')));
if ~bdIsLoaded('g1_planner')
    open_system(fullfile(root,'models','g1_planner.slx'));
end
parent=fullfile(root,'build','block_function_tests');
if ~isfolder(parent),mkdir(parent);end
folder=tempname(parent);mkdir(folder);
cleanup=onCleanup(@()removeTemporaryFunctions(folder,parent));
[~,prefix]=fileparts(folder);prefix=matlab.lang.makeValidName(prefix);
names={'quaternionToEulerZyx','rotateLinearVelocity', ...
    'localAngularVelocityToEulerRates','extractObservation','extractReference'};
blockNames={'quaternionToEulerZYX','linearVelocityToWorld', ...
    'angularVelocityToEulerRates','extractObservation','extractReference'};
rt=sfroot;charts=rt.find('-isa','Stateflow.EMChart');
kernels=struct();
for j=1:numel(names)
    matches=0;
    for k=1:numel(charts)
        if ~startsWith(charts(k).Path,'g1_planner/'),continue;end
        script=charts(k).Script;
        declaration=regexp(script,'^function\s+\w+\s*=\s*(\w+)\s*\(','tokens','once');
        if isempty(declaration) || ~strcmp(declaration{1},blockNames{j}),continue;end
        name=[prefix '_' names{j}];
        script=regexprep(script,['^(function\s+\w+\s*=\s*)' blockNames{j} '\s*\('],['$1' name '(']);
        file=fullfile(folder,[name '.m']);
        fid=fopen(file,'w');assert(fid>=0,'Cannot write temporary test function');
        fprintf(fid,'%s',script);fclose(fid);
        kernels.(names{j})=str2func(name);
        matches=matches+1;
    end
    assert(matches==1,'Expected exactly one conversion block for %s',names{j});
end
addpath(folder);
end

function removeTemporaryFunctions(folder,parent)
assert(startsWith(folder,[parent filesep]),'Temporary directory is outside test cache');
if any(strcmp(strsplit(path,pathsep),folder)),rmpath(folder);end
if isfolder(folder),rmdir(folder,'s');end
end
