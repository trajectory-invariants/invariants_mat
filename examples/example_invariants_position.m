% This script shows how to calculate vector invariants from a given 3D
% position trajectory.

close all; clear; clc;
addpath(genpath('../implementation/'));

%% Settings

%ocp_method = 'regul_movingframes';
ocp_method = 'regul_movingframes_new';
%ocp_method = 'regul_minimumjerk'; 

%init_method = 'average_invariant_frame';
init_method = 'discrete_approx';
%init_method = 'none';

nb_samples = 200;

%% Load measurement data

% Load data. The data are assumed to be structured as [timestamp|pos_x|pos_y|pos_z] (additional columns are ignored)
%filename = '../data/2D_contour_1.txt';
filename = '../data/pouring_motion.csv';
%filename = '../data/sine_wave.txt';

measurement_data = importdata(filename);

% Preprocess and reparameterize
[trajectory,stepsize,arclength] = reparameterize_positiontrajectory(measurement_data,nb_samples);
N = size(trajectory,1);

% plot
figure; axis equal; plot3(trajectory(:,1),trajectory(:,2),trajectory(:,3))

%% Calculate invariants

% Setup symbolic OCP 
params.window.window_length = nb_samples;

if strcmp(ocp_method,'regul_movingframes')
    params.weights.weight_accuracy = 1; % weight on measurement fitting term
    params.weights.weight_regul_deriv_obj = 1e-5; % weight on derivative of object invariants
    params.weights.weight_regul_deriv_mf = 1e-8; % weight on derivative of moving frame invariants
    params.weights.weight_regul_abs_mf = 1e-6; % weight on absolute value of moving frame invariants
    params.weights.scale_rotation = 1; % scaling for making rotations comparable to positions
    params.signed_invariants = 1; % if 1, all invariants are allowed to change sign
    params.positive_obj_invariant = 0;
    params.positive_mov_invariant = 0;

    object = OCP_calculate_vector_invariants_position_old(params);
elseif strcmp(ocp_method,'regul_movingframes_new')
    params.positive_obj_invariant = 1;
    params.positive_mov_invariant = 0;
    params.weights.rms_error_traj = 0.005;

    object = OCP_calculate_vector_invariants_position(params);
elseif strcmp(ocp_method,'regul_minimumjerk')
    params.positive_obj_invariant = 0;
    params.positive_mov_invariant = 0;
    params.weights.weight_accuracy = 1;
    params.weights.weight_regularization = 1e-10;
    object = OCP_calculate_vector_invariants_position_minimumjerk(params);
end

% Initialization of OCP
if strcmp(init_method,'average_invariant_frame')
    twist_init = calculate_posetwist_from_discrete_poses(zeros(3,3,N)+eye(3),trajectory',stepsize);
    [invariants_init, FSt_init] = initialize_invariants_vector(twist_init(:,4:6),params.positive_obj_invariant);
elseif strcmp(init_method,'discrete_approx')
    twist_init = calculate_posetwist_from_discrete_poses(zeros(3,3,N)+eye(3),trajectory',stepsize);

    % Initialize invariants and moving frames over the whole horizon using discretized analytical formulas
    parameters = struct(); % optional arguments
    parameters.signed_invariants = 1; % allow all invariants to become either positive or negative, otherwise omega1 and omega2 are always positive but X-axis and Y-axis may flip
    [FSt_init,~,invariants_init] = calculate_vector_invariants_from_discrete_twist(twist_init,stepsize,parameters);
    invariants_init = invariants_init(:,4:6) +1e-10;
elseif strcmp(init_method,'none')
    FSt_init = zeros(3,3,N) + diag([1 1 1]);
    invariants_init = 1e-7*ones(N,3);
end

% Calculate invariants
optim_class_result = object.calculate_invariants(trajectory,stepsize,invariants_init,FSt_init);
ocp_invariants = optim_class_result.invariants;
ocp_trajectory = optim_class_result.Obj_location;
ocp_movingframes = optim_class_result.FS_frames;

% plot results
plot_vector_invariants(arclength,ocp_invariants,'position')

figure; hold on;
plot3(trajectory(:,1),trajectory(:,2),trajectory(:,3),'b')
plot3(ocp_trajectory(:,1),ocp_trajectory(:,2),ocp_trajectory(:,3),'r')


plot_FS_frames(ocp_movingframes,ocp_trajectory) 

