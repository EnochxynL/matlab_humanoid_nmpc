function g3a_load_messages()
root=fileparts(fileparts(mfilename('fullpath')));
generated=fullfile(root,'build','ros_messages','matlab_msg_gen','glnxa64','install','m');
assert(isfolder(generated),'Build custom bindings using ros2genmsg first.');
addpath(genpath(generated));
rehash toolboxcache;
end
