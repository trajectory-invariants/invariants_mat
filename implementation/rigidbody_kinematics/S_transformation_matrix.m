function S_i_f = S_transformation_matrix(T_i_f)
% This function creates a sequence of screw transformation matrices S from 
% the given pose matrices T so that when applied to a sequence of screws, 
% it transforms the screws from reference frame {i} to reference frame {f}
%
% Input:
%   T_i_f       pose of {i} wrt {f}                                                      [4x4xN]
% Output:
%   S_i_f       screw transformation matrix transforms twist/wrench from {i} to {f}      [6x6xN]

N = size(T_i_f,3);
S_i_f = zeros(6,6,N);

for j = 1 : N
    S_i_f(:,:,j) = [T_i_f(1:3,1:3,j) zeros(3);skew(T_i_f(1:3,4,j)')*T_i_f(1:3,1:3,j) T_i_f(1:3,1:3,j)];
end
