function [R_o2_o1_new,p_o2_o1_new,T_o2_o1_new] = reverse_artificial_variations(T_o2_o1_init,trial)
%%
% This function reverses the applied artificial transformations by function "make_artificial_variations".

% Input
%     R_o2_o1_init:       rotation matrices of artificial data                 [3x3xN]
%     p_o2_o1_init:       position of the artificial data                      [Nx3]
%     trial:              number of trial                                      [1]
% Output
%     R_o2_o1_new:        rotation after reversing artificial transformation   [3x3xN]
%     p_o2_o1_new:        position after reversing artificial transformation   [Nx3]
%     pose_meas:          pose after reversing artificial transformation       [4x4xN]

%%
%T_o2_o1_init = compose_pose_matrix(R_o2_o1_init,p_o2_o1_init);
N = size(T_o2_o1_init,3);
T_o2_o1_new = zeros(4,4,N);

if trial == 1
    R_artificial = roty(-60);
    p_artificial = [-3 2 -1]';
    for j = 1 : N
        T_o2_o1_new(:,:,j)  = inverse_pose([R_artificial,p_artificial;0 0 0 1])*T_o2_o1_init(:,:,j);
    end
elseif trial == 2
    R_artificial = rotz(180)*rotx(-30);
    p_artificial = [-1 2.5 -1]';
    for j = 1 : N
        T_o2_o1_new(:,:,j)  = inverse_pose([R_artificial,p_artificial;0 0 0 1])*T_o2_o1_init(:,:,j);
    end
elseif trial == 3
    R_artificial = rotz(180)*roty(60);
    p_artificial = [-1 2 -2]';
    for j = 1 : N
        T_o2_o1_new(:,:,j)  = inverse_pose([R_artificial,p_artificial;0 0 0 1])*T_o2_o1_init(:,:,j);
    end
elseif trial == 4
    R_artificial = rotx(45);
    p_artificial = [-3 2.5 -2]';
    for j = 1 : N
        T_o2_o1_new(:,:,j)  = inverse_pose([R_artificial,p_artificial;0 0 0 1])*T_o2_o1_init(:,:,j);
    end
elseif trial == 5
    R_artificial = roty(90)*rotx(-45);
    p_artificial = [-1 0.5 0.5]';
    for j = 1 : N
        T_o2_o1_new(:,:,j)  = inverse_pose([R_artificial,p_artificial;0 0 0 1])*T_o2_o1_init(:,:,j);
    end
elseif trial == 6
    R_artificial = roty(90)*rotz(45);
    p_artificial = [1 0 0.5]';
    for j = 1 : N
        T_o2_o1_new(:,:,j)  = inverse_pose([R_artificial,p_artificial;0 0 0 1])*T_o2_o1_init(:,:,j);
    end
elseif trial == 7
    R_artificial = rotx(-90)*rotz(45);
    p_artificial = [1 0.5 -0.5]';
    for j = 1 : N
        T_o2_o1_new(:,:,j)  = inverse_pose([R_artificial,p_artificial;0 0 0 1])*T_o2_o1_init(:,:,j);
    end
elseif trial == 8
    R_artificial = roty(90)*rotz(225);
    p_artificial = [-1 0 -0.5]';
    for j = 1 : N
        T_o2_o1_new(:,:,j)  = inverse_pose([R_artificial,p_artificial;0 0 0 1])*T_o2_o1_init(:,:,j);
    end
elseif trial == 9
    R_artificial = rotx(60)*rotz(210);
    p_artificial = [1 -2 2]';
    for j = 1 : N
        T_o2_o1_new(:,:,j)  = inverse_pose([R_artificial,p_artificial;0 0 0 1])*T_o2_o1_init(:,:,j);
    end
elseif trial == 10
    R_artificial = roty(45)*rotz(-60);
    p_artificial = [3 -2 2]';
    for j = 1 : N
        T_o2_o1_new(:,:,j)  = inverse_pose([R_artificial,p_artificial;0 0 0 1])*T_o2_o1_init(:,:,j);
    end
elseif trial == 11
    R_artificial = roty(-45)*rotz(-180);
    p_artificial = [3 -1.5 1]';
    for j = 1 : N
        T_o2_o1_new(:,:,j)  = inverse_pose([R_artificial,p_artificial;0 0 0 1])*T_o2_o1_init(:,:,j);
    end
elseif trial == 12
    R_artificial = rotx(-60)*rotz(-45);
    p_artificial = [1 -1.5 1]';
    for j = 1 : N
        T_o2_o1_new(:,:,j)  = inverse_pose([R_artificial,p_artificial;0 0 0 1])*T_o2_o1_init(:,:,j);
    end
end

p_o2_o1_new         = squeeze(T_o2_o1_new(1:3,4,:))';
R_o2_o1_new         = T_o2_o1_new(1:3,1:3,:);
end
















