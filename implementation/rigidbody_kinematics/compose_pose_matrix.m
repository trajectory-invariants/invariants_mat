function T_a_b = compose_pose_matrix(R_a_b,p_a_b)
% This function provides transformation matrix, T, contructed based on a
% series of rotation matrices and position vectors.
%
% Notation:
%     p_a_b = position of a wrt b
%     R_a_b = rotation of a wrt b
%     T_a_b = pose of a wrt b
%
% Input:
%     p_a_b: position vector          [N,3]
%     R_a_b: rotation matrix          [3,3,N]
%
% Output:
%     T_a_b: transformation matrix    [4,4,N]

%% Calculation of T

N = size(R_a_b,3);
T_a_b = zeros(4,4,N);

for j = 1 : N
    T_a_b(:,:,j) = [R_a_b(:,:,j),p_a_b(j,:)';0 0 0 1];
end