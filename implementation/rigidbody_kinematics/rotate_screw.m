function twist_transformed = rotate_screw(T_i_f,twist)
% This function creates a transformed twist which is transformed by a
% transformaitoin matrix, P. P transforms ref. frame of a twist from {i} to
% {f} (the leading subscript).
%
% Input:
%   T_i_f                   pose of {i} wrt {f}             [4x4,N]
%   twist                   twist or wrench                 [1x6]
%
% Output:
%   twist_transformed       twist or wrench                 [1x6]

N = size(twist,1);
twist_transformed = zeros(N,6);
for j = 1 : N
    twist_transformed(j,:) = ([T_i_f(1:3,1:3,j) zeros(3,3);zeros(3,3) T_i_f(1:3,1:3,j)]*twist(j,:)')';
end
