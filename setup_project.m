function root=setup_project()
% Local paths and buses; never resets MATLAB or changes controller mathematics.
root=fileparts(mfilename('fullpath'));cd(root);addpath(fullfile(root,'matlab'));addpath(fullfile(root,'models'));
assert(startsWith(which('g1_config'),root),'Another project shadows the extracted files.');
g3a_load_messages();g3a_define_buses();task5a_define_buses();task5b_define_buses();
end
