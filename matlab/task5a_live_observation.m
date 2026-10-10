function s=task5a_live_observation(p)
% Session is transport metadata; numeric observation conversion is explicit.
coder.extrinsic('task5a_session');session=uint64(0);session=task5a_session();s=task5a_observation(p,session);
end
