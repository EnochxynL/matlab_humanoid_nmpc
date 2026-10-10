function cfg = initialize()
%INITIALIZE Add this project's runtime paths without resetting MATLAB state.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root,'matlab'),fullfile(root,'models'), ...
    fullfile(root,'tools'),fullfile(root,'build','controller'));
cfg = g1.loadConfiguration();
end
