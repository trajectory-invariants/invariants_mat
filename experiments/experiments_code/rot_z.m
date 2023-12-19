function R = rot_z(alpha)
%ROT_X Summary of this function goes here
%   Detailed explanation goes here

alpha = alpha*pi/180;
R = [cos(alpha) -sin(alpha) 0 ; sin(alpha) cos(alpha) 0 ; 0 0 1];

end

