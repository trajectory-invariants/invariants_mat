% This script reconstructs motion and force trajectories of the contour
% following task at a new location, and determines the error between the
% reconstructed trajectories and the original measured trajectories (see
% Figure 11 and Table 5 for results).
%
% ! REQUIREMENTS FOR RUNNING THIS SCRIPT:
%
% First run experiments_screw_invariants.m with:
%   - 'contour' as the application
%   - 'motion' as the trajectory type
%   - 'world' as the viewpoint
%   - 'tool_point' as the reference point
% Then run experiments_screw_invariants.m with:
%   - 'contour' as the application
%   - 'wrench' as the trajectory type
%   - 'world' as the viewpoint
%   - 'force_sensor' as the reference point
% The results should be stored automatically in the output folder

close all; clear; clc;
addpath(genpath('../implementation/'));
addpath(genpath('additional_functions/'));
path_to_data_folder = '../data/';


%% First part: Reconstruct the rigid body trajectory of the tool from the calculated screw invariants

%%%% Step 1: Retrieve the (previously calculated) screw invariants of the tool's trajectory 

% used settings 
application = 'contour';
trajectory_type = 'motion'; % {motion,wrench} for screw invariants
viewpoint = 'world';
referencepoint = 'tool_point';

% used tolerances in the OCP
rms_error_pos = 0.002; % [mm]
rms_error_rot = 2*pi/180; % 2 [degrees] converted to [rad]

% select a trial (In the paper, the rigid body trajectory of the tool of trial 7 was chosen)
trial = 7;

% retrieve the stored data
[measured_pose,recons_pose,ISA_frames_pose,screw_invariants_pose,h,N] = retrieve_data_calculated_screw_invariants_motion(application,trajectory_type,viewpoint,referencepoint,trial);


%%% Step 2: Reconstruct the geometric trajectory of the tool by integrating the screw invariants at a new location

% Choose a global transformation. This transformation determines the new location and orientation of the contour.
T_global = design_global_pose_transformation_matrix(measured_pose);

% determine the initial value for the pose of the initial ISA-frame and object-frame (after a applying a global transformation)
transformed_initial_ISA_frame_pose = T_global*ISA_frames_pose(:,:,1); 
transformed_initial_Obj_pose = T_global*recons_pose(:,:,1);

% reconstruct the rigid body trajectory by open-loop integration
[reconstructed_Obj_pose,reconstructed_ISA_frame_motion] = reconstruct_pose_trajectory_from_screw_invariants(...
    screw_invariants_pose,transformed_initial_Obj_pose,transformed_initial_ISA_frame_pose,h,N);

% Apply the same global transformation to the measurements. These transformed measurements serve as a reference to calculate the
% rms-difference between the measured and reconstructed trajectory
transformed_pose_measurements = left_multiply_pose(T_global,measured_pose);

% Calculate the rms differences
[rms_diff_pos,rms_diff_orientation] = calculate_rms_error_pose_trajectory(transformed_pose_measurements,reconstructed_Obj_pose);

rms_diff_delta_pos = rms_diff_pos-rms_error_pos; % Calculate the deviation from the allowed tolerance
rms_diff_delta_orientation = rms_diff_orientation-rms_error_rot; % Calculate the deviation from the allowed tolerance

%% Second part: Reconstruct the wrench trajectory of the contact forces between the tool and the contour from the calculated screw invariants

%%% Step 1: Retrieve the (previously calculated) screw invariants of the wrench trajectory 

% used settings 
application = 'contour';
trajectory_type = 'wrench'; % {motion,wrench} for screw invariants
viewpoint = 'world';
referencepoint = 'force_sensor';
wrenchtype = 'real'; % {real, synthetic}

% used tolerances in the OCP
rms_error_force = 0.8; % [N]
rms_error_moment = 0.16; % [Nm]

%%% Step 2: Reconstruct the wrench trajectory of the contact forces by integrating the screw invariants at a new location

% retrieve the relevant data
[measured_wrench,recons_wrench,ISA_frames_wrench,invars_wrench,h,N] = retrieve_data_calculated_screw_invariants_wrench(application,trajectory_type,viewpoint,referencepoint,trial);

% determine the initial value for the pose of the initial ISA-frame(after a applying a global transformation)
transformed_initial_ISA_frame_wrench = T_global*ISA_frames_wrench(:,:,1);

% reconstruct the contact wrench trajectory by open-loop integration
[reconstructed_wrench,reconstructed_ISA_frame_wrench] = reconstruct_wrench_trajectory_from_screw_invariants(invars_wrench,transformed_initial_ISA_frame_wrench,h,N);

% Transform the reconstructed wrench to the new location of the force-sensor frame before calculating the RMS error:
back_transformed_recons_wrench = zeros(6,N);
for k = 1:N
    back_transformed_recons_wrench(:,k) = transform_screw(inverse_pose(T_global),reconstructed_wrench(k,:)');
end

% Calculate the rms difference
[rms_diff_force,rms_diff_moment] = calculate_rms_error_wrench_trajectory(measured_wrench,back_transformed_recons_wrench');
rms_diff_delta_force = rms_diff_force-rms_error_force; % Calculate the deviation from the allowed tolerance
rms_diff_delta_moment = rms_diff_moment-rms_error_moment; % Calculate the deviation from the allowed tolerance


%% Third part: retrieve the original time profile and re-apply this time profile by reparametrizing the geometric trajectories to temporal trajectories

% Retrieve the original timebased pose measurements 
% (these were resampled to a batch size of N)
referencepoint = 'tool_point';
parameterization = 'time_based';
[time_progress,pose_time] = ...
    contour_preprocess_data(N,viewpoint,parameterization,referencepoint,1,12,application,wrenchtype,path_to_data_folder);

% Retrieve the geometric progress rate
parameterization = 'dimless_arclength';
[geom_progress,pose,position,~,~,~,~,~,~,~,~,~,~,~,velocity_profile,~] = ...
    contour_preprocess_data(N,viewpoint,parameterization,referencepoint,1,12,application,wrenchtype,path_to_data_folder);

% Retrieve the original timebased wrench measurements 
referencepoint = 'force_sensor';
parameterization = 'time_based';
[~,~,~,~,wrench_time] = ...
    contour_preprocess_data(N,viewpoint,parameterization,referencepoint,1,12,application,wrenchtype,path_to_data_folder);

if strcmp(viewpoint,'world')
    wrench_time = -wrench_time;
end

% Apply a global transformation to the measurements
transformed_pose_time = left_multiply_pose(T_global,pose_time(:,:,:,trial));

% Retrieve the original time profile: time(xi)
time_xi = velocity_profile(trial).velocity_profile; % [time xi]
[reconstructed_Obj_pose_time] = apply_time_profile_to_geom_pose_trajectory(reconstructed_Obj_pose,time_xi);
[reconstructed_wrench_time] = apply_time_profile_to_geom_wrench_trajectory(reconstructed_wrench,time_xi);

% Calculate the rms difference
[rms_diff_pos,rms_diff_orientation] = calculate_rms_error_pose_trajectory(transformed_pose_time,reconstructed_Obj_pose_time);

% Transform the reconstructed wrench back to the new location of the force-sensor frame before calculating the RMS error:
back_transformed_recons_wrench_time = zeros(6,N);
for k = 1:N
    back_transformed_recons_wrench_time(:,k) = transform_screw(inverse_pose(T_global),reconstructed_wrench_time(k,:)');
end

% Calculate the rms difference
[rms_diff_force,rms_diff_moment] = calculate_rms_error_wrench_trajectory(wrench_time(:,:,trial),back_transformed_recons_wrench_time');

%% Reporting and plotting of the results

disp(' ')
fprintf('<strong>RMS differences between measurements and open-loop reconstructed trajectories evaluated in geometric domain:</strong>\n')
disp(strcat(['position trajectory:    ' num2str(rms_diff_pos*1000) ' + ' num2str(rms_diff_delta_pos*1000)  ' mm']))
disp(strcat(['orientation trajectory: ' num2str(rms_diff_orientation*180/pi) ' + ' num2str(rms_diff_delta_orientation*180/pi)  ' degrees']))
disp(strcat(['force trajectory:       ' num2str(rms_diff_force) ' + ' num2str(rms_diff_delta_force)  ' Newton']))
disp(strcat(['torque trajectory:      ' num2str(rms_diff_moment) ' + ' num2str(rms_diff_delta_moment) ' Nm']))

disp(' ')
fprintf('<strong>RMS differences between measurements and open-loop reconstructed trajectories evaluated in time domain:</strong>\n')
disp(strcat(['position trajectory:    ' num2str(rms_diff_pos*1000) ' mm']))
disp(strcat(['orientation trajectory: ' num2str(rms_diff_orientation*180/pi) ' degrees']))
disp(strcat(['force trajectory:       ' num2str(rms_diff_force) ' Newton']))
disp(strcat(['torque trajectory:      ' num2str(rms_diff_moment) ' Nm']))

plot_reconstruction_results(pose_time(:,:,:,trial),wrench_time(:,:,trial),T_global,transformed_pose_time,...
    reconstructed_Obj_pose_time,reconstructed_ISA_frame_motion,...
    reconstructed_wrench_time,reconstructed_ISA_frame_wrench,trial)
% exportgraphics(gcf,'../figures/transformation_screw_trajectories.pdf','ContentType','vector');
