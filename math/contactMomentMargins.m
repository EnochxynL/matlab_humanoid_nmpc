function out=contactMomentMargins(bounds,localWrench)
%#codegen
% bounds=[xmin;xmax;ymin;ymax]; packed [f, dx58, du35].
% Original geometry/world-to-local AD is a strict dependency, not migrated.
out=zeros(4,94);
for k=1:94
    fz=localWrench(3,k); mx=localWrench(4,k); my=localWrench(5,k);
    out(1,k)=mx-bounds(3)*fz;
    out(2,k)=-mx+bounds(4)*fz;
    out(3,k)=-my-bounds(1)*fz;
    out(4,k)=my+bounds(2)*fz;
end
end
