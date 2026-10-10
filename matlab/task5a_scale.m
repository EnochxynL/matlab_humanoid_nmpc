function c=task5a_scale(u)
% Original G1 scaling; height is absolute, no added clamp/correction.
c=u;c.values=u.values.*[2;1.2;1;1];
end
