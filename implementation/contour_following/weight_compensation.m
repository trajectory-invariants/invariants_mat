function [wrench_lc_lc_final] = weight_compensation(T_tr_w,wrench_lc_lc_init,T_lc_tr,mass,p_cog_lc)
% This function removes the effects of tool weight from wrench measurement
%
% Notation:
%     p_a_b = position of a wrt b
%     R_a_b = rotation of a wrt b
%     T_a_b = pose of a wrt b
%     w_d_c = wrench between objects, expressed in d, with ref point in c
%
% Input:
%     T_tr_w:               pose of tracker frame {tr} wrt world frame {w}          [4x4x1]
%     wrench_lc_lc_init:    wrench applied to the load cell by contour              [Nx6]
%     T_lc_tr:              pose of load cell frame {lc} wrt tracker frame {tr}     [4x4x1]
%     mass:                 mass of the tool [kg]                                   [1]
%     p_cog_lc:             center of gravity of the tool expressed in {lc}         [1x3]
%
% Output:
%     wrench_lc_lc_final    wrench in weight compensated form                       [Nx6]

%% Parameters of the tool

% mass and p_cog_lc are two inputs.
gravity = 9.81;
% weight of the tool expressed in {w}
force_weight_world  = [0 0 -mass*gravity]';
% moment resulted by weight in {w}
moment_weight_world = [0 0 0]';
% total wrench resulted by weight expressed in {w}
wrench_weight_world = [force_weight_world; moment_weight_world];
% virtual force to remove offset expresesd in {lc}
wrench_virtual_cog  = [-force_weight_world; moment_weight_world];

N = size(T_tr_w,3);
R_lc_w  = zeros(3,3,N);
R_w_lc  = zeros(3,3,N);
for j = 1 : N
    R_lc_w(:,:,j) = T_tr_w(1:3,1:3,j)*T_lc_tr(1:3,1:3);
    R_w_lc(:,:,j) = inv(R_lc_w(:,:,j));
end

% To remove the effect of weight
wrench_compensated_lc = zeros(N,6);
for j = 1 : N
    S_w_lc = S_transformation_matrix(compose_pose_matrix(R_w_lc(:,:,j),p_cog_lc));
    wrench_weight_lc = S_w_lc*wrench_weight_world;
    wrench_compensated_lc(j,:) = wrench_lc_lc_init(j,:)-wrench_weight_lc';
end

% To remove the effect of offset by a virtual force
wrench_modified_lc = zeros(N,6);
S_cog_lc= S_transformation_matrix(compose_pose_matrix(eye(3,3),p_cog_lc));
for j = 1 : N
    wrench_virtual_lc = S_cog_lc*wrench_virtual_cog;
    wrench_modified_lc(j,:) = wrench_compensated_lc(j,:)-wrench_virtual_lc';
end

wrench_lc_lc_final = wrench_modified_lc;












