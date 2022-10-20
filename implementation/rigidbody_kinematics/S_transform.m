function twist_transformed = S_transform(T_i_f,twist)
% This function creates a transformed screw twist which is transformed by a
% screw twist transformaitoin matrix, S. S transforms both ref. frame and
% ref. point from {i} to {f} at the same time (the leading subscript and
% trailing superscript).
%
% Input:
%   T_i_f:                  pose of {i} wrt {f}             [4x4xN]
%   twist                   twist or wrench                 [Nx6]
%
% Output:
%   twist_transformed       twist or wrench                 [Nx6]

%% Calculation
N1 = size(T_i_f,3);
N2 = size(twist,1);

if N1 ~= N2 && N1 == 1
    N = N2;
    twist_transformed = zeros(N,6);
    S = S_transformation_matrix(T_i_f);
    for j = 1 : N
        twist_transformed(j,:) = ( S(:,:,1)*twist(j,:)' )';
    end
elseif N1 == N2
    N = N1;
    twist_transformed = zeros(N,6);
    S = S_transformation_matrix(T_i_f);
    for j = 1 : N
        twist_transformed(j,:) = ( S(:,:,j)*twist(j,:)' )';
    end
end

% N1 = size(T,3);
% N2 = size(screw_twist,2);
%
% if N1 ~= N2 && N1 == 1
%     N = N2;
%     screw_twist_transformed = zeros(6,N);
%     for j = 1 : N
%         screw_twist_transformed(:,j) = [T(1:3,1:3)*screw_twist(1:3,j) ;
%                         T(1:3,1:3)*screw_twist(4:6,j) + cross(T(1:3,4),T(1:3,1:3)*screw_twist(1:3,j)) ];
%     end
% elseif N1 == N2
%     N = N1;
%     screw_twist_transformed = zeros(6,N);
%     for j = 1 : N
%         screw_twist_transformed(:,j) = [T(1:3,1:3,j)*screw_twist(1:3,j) ;
%                         T(1:3,1:3,j)*screw_twist(4:6,j) + cross(T(1:3,4,j),T(1:3,1:3,j)*screw_twist(1:3,j)) ];
%     end
% end
