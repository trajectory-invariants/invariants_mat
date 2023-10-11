function [weight_calib,T_tcp_tr,T_tcp_lc] = configuration_properties_contour(trial)
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


% number identifying the location and orientation of the force sensor {lc} and motion tracker {tr} on the tool
sensor_id = configurations();
configuration = sensor_id(trial);
mass = 1.96138/9.806; % mass of tool in [kg] determined with a least-squares calibration procedure
%%
if strcmp(configuration,'a')
    % Rotation {tcp} wrt {tr}
    R_tcp_tr = rotx(135);
    % Position {tcp} wrt {tr}
    p_tcp_tr = [0,-99.69,35.5]./1000;
    % Pose {tcp} wrt {tr}
    T_tcp_tr = [R_tcp_tr,p_tcp_tr';0 0 0 1];
    
    % Rotation {tcp} wrt {lc}
    R_tcp_lc = rotz(-90)*rotx(225); % {lc} towards {tcp} R1 * R2
    % Position {tcp} wrt {lc}
    p_tcp_lc = [0,0,-55.17]./1000;
    % Pose {tcp} wrt {lc}
    T_tcp_lc = [R_tcp_lc,p_tcp_lc';0 0 0 1];
    
    % Pose {lc} wrt {tcp}
    T_lc_tcp = inverse_pose(T_tcp_lc);
    % Pose {lc} wrt {tr}
    T_lc_tr = T_tcp_tr*T_lc_tcp;
    
    % Center of gravity (COG) of the tool in [m] determined with a least-squares calibration procedure expressed in {lc}
    p_cog_lc = [0.00181208, 0.000785471, -0.0273515];

elseif strcmp(configuration,'b')
    % Rotation {tcp} wrt {tr}
    R_tcp_tr = roty(180)*rotx(135); % {tr} towards {tcp} R1 * R2
    % Position {tcp} wrt {tr}
    p_tcp_tr = [0,-99.69,35.5]./1000;
    % Pose {tcp} wrt {tr}
    T_tcp_tr = [R_tcp_tr,p_tcp_tr';0 0 0 1];
    
    % Rotation {tcp} wrt {lc}
    R_tcp_lc = rotz(-90)*rotx(225); % {lc} towards {tcp} R1 * R2
    % Position {tcp} wrt {lc}
    p_tcp_lc = [0,0,-55.17]./1000;
    % Pose {tcp} wrt {lc}
    T_tcp_lc = [R_tcp_lc,p_tcp_lc';0 0 0 1];
    
    % Pose {lc} wrt {tcp}
    T_lc_tcp = inverse_pose(T_tcp_lc);
    % Pose {lc} wrt {tr}
    T_lc_tr = T_tcp_tr*T_lc_tcp;
    
    % Center of gravity (COG) of the tool in [m] determined with a least-squares calibration procedure expressed in {lc}
    p_cog_lc = [0.00181208, 0.000785471, -0.0273515];

elseif strcmp(configuration,'c')
    % Rotation {tcp} wrt {tr}
    R_tcp_tr = rotx(135);
    % Position {tcp} wrt {tr}
    p_tcp_tr = [0,-261.69,35.5]./1000;
    % Pose {tcp} wrt {tr}
    T_tcp_tr = [R_tcp_tr,p_tcp_tr';0 0 0 1];
    
    % Rotation {tcp} wrt {lc}
    R_tcp_lc = rotz(-90)*rotx(225); % {lc} towards {tcp} R1 * R2
    % Position {tcp} wrt {lc}
    p_tcp_lc = [0,0,-55.17]./1000;
    % Pose {tcp} wrt {lc}
    T_tcp_lc = [R_tcp_lc,p_tcp_lc';0 0 0 1];
    
    % Pose {lc} wrt {tcp}
    T_lc_tcp = inverse_pose(T_tcp_lc);
    % Pose {lc} wrt {tr}
    T_lc_tr = T_tcp_tr*T_lc_tcp;
    
    % Center of gravity (COG) of the tool in [m] determined with a least-squares calibration procedure expressed in {lc}
    p_cog_lc = [0.00181208, 0.000785471, -0.0273515];

elseif strcmp(configuration,'d')
    % Rotation {tcp} wrt {tr}
    R_tcp_tr = roty(90)*rotx(135); % {tr} towards {tcp} R1 * R2
    % Position {tcp} wrt {tr}
    p_tcp_tr = [0,-261.69,35.5]./1000;
    % Pose {tcp} wrt {tr}
    T_tcp_tr = [R_tcp_tr,p_tcp_tr';0 0 0 1];
    
    % Rotation {tcp} wrt {lc}
    R_tcp_lc = rotz(-90)*rotx(225); % {lc} towards {tcp} R1 * R2
    % Position {tcp} wrt {lc}
    p_tcp_lc = [0,0,-55.17]./1000;
    % Pose {tcp} wrt {lc}
    T_tcp_lc = [R_tcp_lc,p_tcp_lc';0 0 0 1];
    
    % Pose {lc} wrt {tcp}
    T_lc_tcp = inverse_pose(T_tcp_lc);
    % Pose {lc} wrt {tr}
    T_lc_tr = T_tcp_tr*T_lc_tcp;
    
    % Center of gravity (COG) of the tool in [m] determined with a least-squares calibration procedure expressed in {lc}
    p_cog_lc = [0.00181208, 0.000785471, -0.0273515];

elseif strcmp(configuration,'e')
    % Rotation {tcp} wrt {tr}
    R_tcp_tr = roty(180)*rotx(135); % {tr} towards {tcp} R1 * R2
    % Position {tcp} wrt {tr}
    p_tcp_tr = [0,-261.69,35.5]./1000;
    % Pose {tcp} wrt {tr}
    T_tcp_tr = [R_tcp_tr,p_tcp_tr';0 0 0 1];
    
    % Rotation {tcp} wrt {lc}
    R_tcp_lc = rotz(-90)*rotx(225); % {lc} towards {tcp} R1 * R2
    % Position {tcp} wrt {lc}
    p_tcp_lc = [0,0,-55.17]./1000;
    % Pose {tcp} wrt {lc}
    T_tcp_lc = [R_tcp_lc,p_tcp_lc';0 0 0 1];
    
    % Pose {lc} wrt {tcp}
    T_lc_tcp = inverse_pose(T_tcp_lc);
    % Pose {lc} wrt {tr}
    T_lc_tr = T_tcp_tr*T_lc_tcp;
    
    % Center of gravity (COG) of the tool in [m] determined with a least-squares calibration procedure expressed in {lc}
    p_cog_lc = [0.00181208, 0.000785471, -0.0273515];

elseif strcmp(configuration,'f')
    % Rotation {tcp} wrt {tr}
    R_tcp_tr = roty(-90)*rotx(135); % {tr} towards {tcp} R1 * R2
    % Position {tcp} wrt {tr}
    p_tcp_tr = [0,-261.69,35.5]./1000;
    % Pose {tcp} wrt {tr}
    T_tcp_tr = [R_tcp_tr,p_tcp_tr';0 0 0 1];
    
    % Rotation {tcp} wrt {lc}
    R_tcp_lc = rotz(-90)*rotx(225); % {lc} towards {tcp} R1 * R2
    % Position {tcp} wrt {lc}
    p_tcp_lc = [0,0,-55.17]./1000;
    % Pose {tcp} wrt {lc}
    T_tcp_lc = [R_tcp_lc,p_tcp_lc';0 0 0 1];
    
    % Pose {lc} wrt {tcp}
    T_lc_tcp = inverse_pose(T_tcp_lc);
    % Pose {lc} wrt {tr}
    T_lc_tr = T_tcp_tr*T_lc_tcp;
    
    % Center of gravity (COG) of the tool in [m] determined with a least-squares calibration procedure expressed in {lc}
    p_cog_lc = [0.00181208, 0.000785471, -0.0273515];
end

weight_calib.mass = mass;
weight_calib.T_lc_tr = T_lc_tr;
weight_calib.p_cog_lc = p_cog_lc;
