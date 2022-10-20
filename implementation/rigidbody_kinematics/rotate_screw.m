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

N1 = size(T_i_f,3);
N2 = size(twist,1);

if N1 ~= N2 && N1 == 1
    N = N2;
    twist_transformed = zeros(N,6);
    P_i_f = P_transformation_matrix(T_i_f);
    for j = 1 : N
        twist_transformed(j,:) = (P_i_f*twist(j,:)')';
    end
elseif N1 == N2
    N = N1;
    twist_transformed = zeros(N,6);
    P_i_f = P_transformation_matrix(T_i_f);
    for j = 1 : N
        twist_transformed(j,:) = (P_i_f(:,:,j)*twist(j,:)')';
    end
end
end

%%
function P_i_f = P_transformation_matrix(T_i_f)
% This function creates a transformation matrix, P, which transforms ref.
% frame of a twist from {i} to {f} (the leading subscript)
%
% Input:
%   T_i_f       pose of {i} wrt {f}                                                      [4x4xN]
% Output:
%   P_i_f       transformation matrix transforms twist/wrench from {i} to {f}            [6x6xN]

N = size(T_i_f,3);
P_i_f = zeros(6,6,N);
for j = 1 : N
    P_i_f(:,:,j) = [T_i_f(1:3,1:3,j) zeros(3);zeros(3) T_i_f(1:3,1:3,j)];
end
end



% function twist_transf = pscrew_trans_ali(T,twist)
%
% % this function creates a transformed twist which
% % is transformed by a transformaitoin matrix (P) to another frame
%
% % input: transformation matrix [4x4], T but only R is used
% % input: transformation matrix [1x6], twist
% % outpu: transformed twist [1x6], twist_transf
%
% N1 = size(T,3);
% N2 = size(twist,1);
%
% R = T(1:3,1:3,:);
%
% if N1 ~= N2 && N1 == 1
%     N = N2;
%     P = zeros(6,6,N);
%     twist_transf = zeros(N,6);
%     for j = 1 : N
%         P(:,:,1) = [R(:,:,1) zeros(3);zeros(3) R(:,:,1)];
%         twist_transf(j,:) = ( P(:,:,1)*twist(j,:)' )';
%     end
% elseif N1 == N2
%     N = N1;
%     P = zeros(6,6,N);
%     twist_transf = zeros(N,6);
%     for j = 1 : N
%         P(:,:,j) = [R(:,:,j) zeros(3);zeros(3) R(:,:,j)];
%         twist_transf(j,:) = ( P(:,:,j)*twist(j,:)' )';
%     end
% end
