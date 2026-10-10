function result = footLinearResidual(b,Ax,Av,Aa,enabled,position,velocity,acceleration)
%#codegen
% Strict original end-effector dependency supplies f/dx/du, packed 6x94.
% Original consumers use diagonal matrices (stance) or one z selector (swing).
% Preserve contribution order: position, velocity, acceleration. No GN change.
result = zeros(6,94);
result(:,1) = b;
for k = 1:3
    if enabled(k)
        if k==1
            A=Ax; data=position;
        elseif k==2
            A=Av; data=velocity;
        else
            A=Aa; data=acceleration;
        end
        for column=1:94
            for row=1:6
                product=0;
                for inner=1:6
                    product=product+A(row,inner)*data(inner,column);
                end
                result(row,column)=result(row,column)+product;
            end
        end
    end
end
end
