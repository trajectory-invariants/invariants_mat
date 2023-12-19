function R = rot_x(alpha)
%ROT_X Summary of this function goes here
%   Detailed explanation goes here

alpha = alpha*pi/180;
R = [1 0 0 ; 0 cos(alpha) -sin(alpha) ; 0 sin(alpha) cos(alpha)];

end

