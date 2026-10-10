function out=collisionMargins(positions,radii)
%#codegen
% Exact original point order: ankle L/R, center L/R, p1 L/R, p2 L/R, knee L/R.
% Original runtime uses Euclidean norm, despite a squared-distance comment.
pairs=[5,6;5,8;7,6;7,8;3,6;3,8;4,5;4,7;3,4;9,10;3,2;5,2;7,2;4,1;6,1;8,1];
out=zeros(16,1);footDistance=2*radii(1);kneeDistance=2*radii(2);
for k=1:16
 d=positions(:,pairs(k,1))-positions(:,pairs(k,2));
 % Eigen fixed vector3 sum reduction: coefficient 0 + (coefficient 1 + 2).
 distance=sqrt(d(1)*d(1)+(d(2)*d(2)+d(3)*d(3)));
 if k==10,out(k)=distance-kneeDistance;else,out(k)=distance-footDistance;end
end
end
