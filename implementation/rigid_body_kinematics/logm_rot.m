function omega_skew = logm_rot(R)
% Returns the matrix logarithm of the given rotation matrix R
%
% Input:
%   R = a (3x3) rotation matrix
% Output:
%   omega_skew = a (3x3) skew-symmetric matrix representing 
%
% Note: better than doing logm(R) since it is a closed-form analytic expression
%

axis_angle = [R(3,2)-R(2,3) ; R(1,3)-R(3,1) ; R(2,1)-R(1,2)] / 2;
sin_angle = norm(axis_angle);
cos_angle = (trace(R)-1)/2;

if cos_angle < -1
    cos_angle = -1;
elseif cos_angle > 1
    cos_angle = 1;
end

if sin_angle < 1e-14
    alpha = 1/2;
else
    alpha = atan2(sin_angle,cos_angle)/(2*sin_angle);
end

omega_skew = (R-R')*alpha;




