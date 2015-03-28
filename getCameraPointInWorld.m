function p_in_world = getCameraPointInWorld(  cKw, p_in_c )
%GE Summary of this function goes here
%   Detailed explanation goes here
p_in_world = cKw\p_in_c; %inv(cKw) * p_in_c; 
end

