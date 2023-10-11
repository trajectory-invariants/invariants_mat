function [R_o2_o1_new,p_o2_o1_new,pose_meas] = make_artificial_variations(R_o2_o1_init,p_o2_o1_init,trial)
% This function contains artificial transformations to challenge invariance of the proposed invariant descriptors.
% 
% Notation:
%     p_a_b = position of a wrt b
%     R_a_b = rotation of a wrt b
%     T_a_b = pose of a wrt b
%     w_d_c = wrench between objects, expressed in d, with ref point in c
% 
% Input
%     R_o2_o1_init:        rotation matrices of measured data                  [3x3xN]
%     p_o2_o1_init:        position of the measured data                       [Nx3]
%     trial:               number of trial                                     [1]
% 
% Output
%     R_o2_o1_new:         rotation after applying artificial transformation   [3x3xN]
%     p_o2_o1_new:         position after applying artificial transformation   [Nx3]
%     pose_meas:           pose after applying artificial transformation       [4x4xN]

%%
T_o2_o1_init = compose_pose_matrix(R_o2_o1_init,p_o2_o1_init);
N = size(T_o2_o1_init,3);
T_o2_o1_new = zeros(4,4,N);

if trial == 1
    R_artificial = roty(-60);
    %euler_angles = rotm2eul(R_artificial,'xyz')*180/pi;
    p_artificial = [-3 2 -1];
    for j = 1 : N
        T_o2_o1_new(:,:,j) = compose_pose_matrix(R_artificial,p_artificial)*T_o2_o1_init(:,:,j);
    end
elseif trial == 2
    R_artificial = rotz(180)*rotx(-30);
    %euler_angles = rotm2eul(R_artificial,'xyz')*180/pi;
    p_artificial = [-1 2.5 -1];
    for j = 1 : N
        T_o2_o1_new(:,:,j) = compose_pose_matrix(R_artificial,p_artificial)*T_o2_o1_init(:,:,j);
    end
elseif trial == 3
    R_artificial = rotz(180)*roty(60);
    %euler_angles = rotm2eul(R_artificial,'xyz')*180/pi;
    p_artificial = [-1 2 -2];
    for j = 1 : N
        T_o2_o1_new(:,:,j) = compose_pose_matrix(R_artificial,p_artificial)*T_o2_o1_init(:,:,j);
    end
elseif trial == 4
    R_artificial = rotx(45);
    %euler_angles = rotm2eul(R_artificial,'xyz')*180/pi;
    p_artificial = [-3 2.5 -2];
    for j = 1 : N
        T_o2_o1_new(:,:,j) = compose_pose_matrix(R_artificial,p_artificial)*T_o2_o1_init(:,:,j);
    end
elseif trial == 5
    R_artificial = roty(90)*rotx(-45);
    %euler_angles = rotm2eul(R_artificial,'xyz')*180/pi;
    p_artificial = [-1 0.5 0.5];
    for j = 1 : N
        T_o2_o1_new(:,:,j) = compose_pose_matrix(R_artificial,p_artificial)*T_o2_o1_init(:,:,j);
    end
elseif trial == 6
    R_artificial = roty(90)*rotz(45);
    %euler_angles = rotm2eul(R_artificial,'xyz')*180/pi;
    p_artificial = [1 0 0.5];
    for j = 1 : N
        T_o2_o1_new(:,:,j) = compose_pose_matrix(R_artificial,p_artificial)*T_o2_o1_init(:,:,j);
    end
elseif trial == 7
    R_artificial = rotx(-90)*rotz(45);
    %euler_angles = rotm2eul(R_artificial,'xyz')*180/pi;
    p_artificial = [1 0.5 -0.5];
    for j = 1 : N
        T_o2_o1_new(:,:,j) = compose_pose_matrix(R_artificial,p_artificial)*T_o2_o1_init(:,:,j);
    end
elseif trial == 8
    R_artificial = roty(90)*rotz(225);
    %euler_angles = rotm2eul(R_artificial,'xyz')*180/pi;
    p_artificial = [-1 0 -0.5];
    for j = 1 : N
        T_o2_o1_new(:,:,j) = compose_pose_matrix(R_artificial,p_artificial)*T_o2_o1_init(:,:,j);
    end
elseif trial == 9
    R_artificial = rotx(60)*rotz(210);
    %euler_angles = rotm2eul(R_artificial,'xyz')*180/pi;
    p_artificial = [1 -2 2];
    for j = 1 : N
        T_o2_o1_new(:,:,j) = compose_pose_matrix(R_artificial,p_artificial)*T_o2_o1_init(:,:,j);
    end
elseif trial == 10
    R_artificial = roty(45)*rotz(-60);
    %euler_angles = rotm2eul(R_artificial,'xyz')*180/pi;
    p_artificial = [3 -2 2];
    for j = 1 : N
        T_o2_o1_new(:,:,j) = compose_pose_matrix(R_artificial,p_artificial)*T_o2_o1_init(:,:,j);
    end
elseif trial == 11
    R_artificial = roty(-45)*rotz(-180);
    %euler_angles = rotm2eul(R_artificial,'xyz')*180/pi;
    p_artificial = [3 -1.5 1];
    for j = 1 : N
        T_o2_o1_new(:,:,j) = compose_pose_matrix(R_artificial,p_artificial)*T_o2_o1_init(:,:,j);
    end
elseif trial == 12
    R_artificial = rotx(-60)*rotz(-45);
    %euler_angles = rotm2eul(R_artificial,'xyz')*180/pi;
    p_artificial = [1 -1.5 1];
    for j = 1 : N
        T_o2_o1_new(:,:,j) = compose_pose_matrix(R_artificial,p_artificial)*T_o2_o1_init(:,:,j);
    end
end

p_o2_o1_new = squeeze(T_o2_o1_new(1:3,4,:))';
R_o2_o1_new = T_o2_o1_new(1:3,1:3,:);
pose_meas.Obj_location = p_o2_o1_new;
pose_meas.Obj_frames = R_o2_o1_new;
end















