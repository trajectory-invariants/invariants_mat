function [T_tcp_tr,T_tcp_lc,T_lc_tr,p_cog_lc] = configuration_properties_peg()
% This function provides the geometric properties of the tool.
%
% Notation:
%     p_a_b = position of a wrt b
%     R_a_b = rotation of a wrt b
%     T_a_b = pose of a wrt b
%     w_d_c = wrench between objects, expressed in d, with ref point in c
%
% Input:
%     configuration:    configuration of the setup (explained in Fig. 4 and TABLE IV of the paper)        [string]
%
% Output:
%     T_tcp_tr:         pose of {tcp} wrt {tr}              [4x4x1]
%     T_tcp_lc:         pose of {tcp} wrt {lc}              [4x4x1]
%     T_lc_tr:          pose of {lc} wrt {tr}               [4x4x1]
%     p_cog_lc:         position of {cog} wrt {lc}          [1,3]

%%
% Rotation  {tcp} wrt {tr}
R_tcp_tr = roty(180);
% Position  {tcp} wrt {tr}
p_tcp_tr = [0, -217.5, 45.5]./1000;
% Pose      {tcp} wrt {lc}
T_tcp_tr = [R_tcp_tr, p_tcp_tr'; 0 0 0 1];

% Rotation  {tcp} wrt {lc}
R_tcp_lc = rotz(45)*rotx(90); % {lc} towards {tcp} R1 * R2
% Position  {tcp} wrt {lc}
p_tcp_lc = [0.0,0.0,-170.5]./1000;
% Pose      {tcp} wrt {lc}
T_tcp_lc = [R_tcp_lc, p_tcp_lc'; 0 0 0 1];

% Pose {lc} wrt {tcp}
T_lc_tcp = inverse_pose(T_tcp_lc);
% Pose {lc} wrt {tr}
T_lc_tr  = T_tcp_tr*T_lc_tcp;


% Center of gravity (COG) of the tool in [m] determined with a least-squares calibration procedure expressed in {lc}
p_cog_lc = [-0.000250788, 0.00179623, -0.0435305]; % optimization
