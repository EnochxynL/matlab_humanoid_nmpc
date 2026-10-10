function u=nominalInput(mass,contactFlags) %#codegen
% Original model mass is an immutable Pinocchio dependency; world wrench35.
force=mass*9.81;count=double(contactFlags(1))+double(contactFlags(2));u=zeros(35,1);
if count>0
 for k=1:2
  if contactFlags(k),u(6*(k-1)+3)=force/count;end
 end
end
end
