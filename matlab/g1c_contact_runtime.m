function y=g1c_contact_runtime(u)
%G1C_CONTACT_RUNTIME Parameters are explicit signal inputs, not base globals.
y=g1c_contact_vector(u(1:56),u(57),u(58),u(59));
% Interpreted MATLAB Function requires finite outputs. Zero is an invalid
% placeholder only; cop_valid=false means no CoP is published.
if ~y(21), y(19:20)=0; end
end
