function output=g3a_runtime(u)
global g3a_bridge
assert(~isempty(g3a_bridge)&&isvalid(g3a_bridge),'ROS adapter requires g3a_start.');
output=g3a_bridge.exchange(u);
end
