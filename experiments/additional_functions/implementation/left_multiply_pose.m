function T_transformed = left_multiply_pose(T_transform,T)
% Apply a global transformation to a pose trajectory by left multiplication
%
% Input:  T_transform      = global pose transformation matrix (4x4)
%         T                = to be transformed pose trajectory (4x4xN)
% Output: T_transformed    = transformed pose trajectory (4x4xN)

    N = size(T,3);
    T_transformed = zeros(4,4,N);
    for k = 1:N
        T_transformed(:,:,k) = T_transform*T(:,:,k);
    end
end