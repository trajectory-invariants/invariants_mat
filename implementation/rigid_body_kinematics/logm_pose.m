function dtwist = logm_pose(T)
%%
% Returns the matrix logarithm of the given homogeneous transformation matrix, T
%
% Input:
%   T = a (4x4) homogeneous transformation matrix matrix [R p ; 0 0 0 1]
% Output:
%   dtwist = a (4x4) displacement twist [skew(deltatheta) deltap ; 0 0 0 0]
%
% Note: better than doing logm(T) since it is a closed-form analytic expression
%
%

R=T(1:3,1:3);
p=T(1:3,4);
dtwist = zeros(4,4);

omega_hat = logm_rot(R);
omega = [-omega_hat(2,3) ; omega_hat(1,3) ; -omega_hat(1,2)];
theta = norm(omega);

if theta < 1e-14
    dtwist(1:3,4) = p;
else

    G = (eye(3)-R)*omega_hat/theta + omega*omega'/theta;

    dtwist(1:3,1:3) = omega_hat;
    dtwist(1:3,4)  = G^(-1)*p*theta;
end
