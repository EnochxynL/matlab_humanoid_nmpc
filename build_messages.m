function build_messages()
% Generate only the custom message package, independent of C++ controller deps.
root=fileparts(mfilename('fullpath'));cd(root);
bindingRoot=fullfile(root,'build','ros_messages');
if ~isfolder(bindingRoot),mkdir(bindingRoot);end
package=fullfile(bindingRoot,'g1_plant_bridge_msgs');
if ~isfolder(package),copyfile(fullfile(root,'ros2_ws','src','g1_plant_bridge_msgs'),package);end
ros2genmsg(bindingRoot);
addpath(fullfile(root,'matlab'));g3a_load_messages();
end
