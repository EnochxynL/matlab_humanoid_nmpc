function session=task5a_session()
% Metadata only: no wall-clock value enters reference or gait equations.
global g3a_bridge
assert(~isempty(g3a_bridge)&&isvalid(g3a_bridge));session=g3a_bridge.Session;
end
