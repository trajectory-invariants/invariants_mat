function Tinv = inverse_pose(T)
%%
% Returns the inverse of the given homogeneous transformation matrix, T.
%
% Input:
%   T = pose matrices, can be either a [4x4xN] matrix [R p ; 0 0 0 1] or a [3x4xN] matrix [R p]
% Output:
%   Tinv = the inverse of T, either a [4x4xN] matrix or a [3x4xN] matrix depending on T
%
% Note: better than doing T^(-1) since it is a closed-form analytic expression

%% Initialization
N = size(T,3);
Tinv = T;

%% Calculation
for j = 1 : N
    R = T(1:3,1:3,j);
    p = T(1:3,4,j);
    Tinv(1:3,:,j) = [transpose(R) -transpose(R)*p];
end



