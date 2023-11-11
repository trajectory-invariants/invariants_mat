% This script allows calculating invariant descriptors and moving frames 
% from the recorded data of the contour following experiment
%
% The following settings need to be chosen to generate the figures of the paper
%
% Figure 6: 
%	Figure 9a: trajectory_type = 'pose', ref_point_motion = 'tracker'
%	Figure 9b: trajectory_type = 'rotation', ref_point_motion = 'tracker'
%	Figure 9c: trajectory_type = 'position', ref_point_motion = 'tracker'
%	Figure 9d: trajectory_type = 'position', ref_point_motion = 'tracker'
% Figure 7: 
%	Figure 7a: trajectory_type = 'wrench', ref_point_motion = 'tracker'
%	Figure 7b: trajectory_type = 'force', ref_point_motion = 'tracker'
%	Figure 7c: trajectory_type = 'moment', ref_point_motion = 'tracker'
%	Figure 7d: trajectory_type = 'moment', ref_point_motion = 'tracker'
% Figure 8: trajectory_type = 'wrench', ref_frame_force = ''
% Figure 9: trajectory_type = 'wrench' ref_frame_force = ''
% Figure 10:
%	Figure 10a: trajectory_type = 'pose', ref_point_motion = 'tracker'
%	Figure 10b: trajectory_type = 'wrench', ref_point_motion = 'tracker'
%	Figure 10c: trajectory_type = 'moment', ref_point_motion = 'tracker'
%	Figure 10d: trajectory_type = 'position', ref_point_motion = 'tracker'

close all; clear; clc;
addpath(genpath('../implementation/'));
addpath(genpath('./experiments_code/'));

% Settings - analysis
settings_analysis.trajectory_type = 'pose'; % {pose,rotation,position,wrench,force,moment}
settings_analysis.ref_point_motion = 'tracker'; % {tracker, tool_point}
settings_analysis.ref_frame_force = 'tracker'; % {tracker, tool_point, under_contour}
settings_analysis.wrench_synthetic = false; % replace real contact wrench with synthetic data
settings_analysis.progress_choice = 'arclength'; % {time,arclength,arcangle}
settings_analysis.N = 101; % number of samples in one trial
% Choose trial_0 = trial_n = X, to only show results of trial X
settings_analysis.trial_0 = 1; % number of first trial to consider {1-12}
settings_analysis.trial_n = 1; % number of final trial to consider {1-12}
settings_analysis.velocity_translation_threshold = 0.05; % threshold on translational velocity [m/s]
settings_analysis.velocity_rotation_threshold = 0.35; % threshold on rotational velocity [rad/s]
settings_analysis.artificial_variations = true;
settings_analysis.application = 'contour'; % {contour,peg}

% Parameters in optimal control problems
parameters_OCP.weights.rms_error_orientation = 2*pi/180; % 2 [degrees] converted to [rad]
parameters_OCP.weights.rms_error_translation = 0.002; % [mm]
parameters_OCP.weights.rms_error_force = 0.8; % [N]
parameters_OCP.weights.rms_error_moment = 0.16; % [Nm]
parameters_OCP.weights.L = 0.5; % [m] global scale to weigh the rotational and translational moving frame invariants
parameters_OCP.regul_origin_ASA = 10^(-10); % Used to regulate the origin of the ASA-frame which is used for the initialization of the OCP
parameters_OCP.max_iters = 500; % maximum number of iterations
parameters_OCP.window.window_length = settings_analysis.N;
parameters_OCP.positive_obj_invariant = 0; 
parameters_OCP.positive_mov_invariant = 0;

% Settings - plots
settings_plots.plot_paper_figures = 0;                     % {0,1}
settings_plots.plot_reference_results = 1;                 % {0,1}
settings_plots.plot_summary_invariants = 1;                % {0,1}
settings_plots.plot_all_trials_movingframes = 0;           % {0,1}
settings_plots.plot_all_trials_trajectory_errors = 1;      % {0,1}
settings_plots.plot_all_trials_invariants = 0;             % {0,1}

%% Load and preprocess data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The data has the following structure:
% data{trial_number}.t = timestamps [Nx1]
% data{trial_number}.pos = position coordinates of tracker attached to tool wrt world [Nx3]
% data{trial_number}.quat = quaternion coordinates of tool wrt world [Nx4]
% data{trial_number}.rotm = rotation matrix of tool wrt world [3x3xN]
% data{trial_number}.force = force vector expressed in load cell frame [Nx3]
% data{trial_number}.torque = moment vector wrt and expressed in load cell frame [Nx3]
%
% The preprocessed data has the following structure:
% preprocessed{trial_number}.progress = equidistant task progress variable [1xN]
% preprocessed{trial_number}.pose = pose matrix of tool with specified reference point wrt world[4x4xN]
% preprocessed{trial_number}.position = position vector of specified reference point wrt world [Nx3]
% preprocessed{trial_number}.rotation = rotation matrix of tool wrt world [3x3xN]
% preprocessed{trial_number}.wrench = contact wrench expressed in specified frame [Nx6]
% preprocessed{trial_number}.force = contact force expressed in specified frame [Nx3]
% preprocessed{trial_number}.moment = contact moment expressed in specified frame [Nx3]
% preprocessed{trial_number}.vel_prof = task progress in function of time and velocity profile [Nx2]

% Real measurement data
path_to_data = 'data/contour_following/measurements';
data_exp = load_trials_in_folder(path_to_data);
measurement_data = preprocess_contour_measurement_data(data_exp,settings_analysis);

% Synthetic reference data
path_to_data = 'data/contour_following/reference';
raw_data_reference = load_trials_in_folder(path_to_data);
reference_data = preprocess_reference_data(raw_data_reference{1},settings_analysis);

%% Calculate invariants of all measured data using optimal control (OCP) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Specify OCP symbolically
OCP = specify_optimal_control_problem(parameters_OCP,settings_analysis.trajectory_type);

% Calculate invariants for each trial 
nb_trials = settings_analysis.trial_n - settings_analysis.trial_0 + 1; % number of trials
for trial=1:nb_trials
    disp(['calculating trial ' num2str(trial) '/' num2str(nb_trials) ' ...']);

    % Select measurements
    [measured_trajectory,progress] = select_measurements(measurement_data{trial},settings_analysis.trajectory_type);
	stepsize = mean(diff(progress)); % stepsize

	% Calculate invariants
    OCP_results = OCP.calculate_invariants(measured_trajectory,stepsize);

    % Store results
    results.trials(trial).progress = progress;
    results.trials(trial).measured_trajectory = measured_trajectory;
    results.trials(trial).invariants = OCP_results.invariants;
    results.trials(trial).reconstructed_trajectory = OCP_results.reconstruction;
    results.trials(trial).moving_frames = OCP_results.moving_frames;
end

%% Calculate invariants of reference trial using optimal control (OCP) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp('calculating reference trial');

% Select measurements
[reference_trajectory,progress] = select_measurements(reference_data,settings_analysis.trajectory_type);
stepsize = mean(diff(progress)); % stepsize

% Calculate invariants
OCP_results = OCP.calculate_invariants(reference_trajectory,stepsize);

% Store results
results.reference.progress = progress;
results.reference.measured_trajectory = reference_trajectory;
results.reference.invariants = OCP_results.invariants;
results.reference.reconstructed_trajectory = OCP_results.reconstruction;
results.reference.moving_frames = OCP_results.moving_frames;

%% Plotting results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

plot_all_results(results,settings_analysis,settings_plots)

save_results(results,settings_analysis,settings_plots)
