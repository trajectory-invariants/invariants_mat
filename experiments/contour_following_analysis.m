%% Contour motion

close all; clear; clc;
addpath(genpath('../implementation/'));
addpath(genpath('./experiments_code/'));

% Settings - analysis
settings_analysis.trajectory_type = 'pose'; % {pose,rotation,position,wrench,force,moment}
settings_analysis.viewpoint = 'world'; % {world, body}
settings_analysis.ref_point_motion = 'tracker'; % {tracker, tool_point}
settings_analysis.ref_frame_force = 'tracker'; % {tracker, tool_point, under_contour}
settings_analysis.progress_choice = 'arclength'; % {time,arclength,arcangle}
settings_analysis.N = 101; % number of samples in one trial
settings_analysis.trial_0 = 1; % {1-12}
settings_analysis.trial_n = 12; % {1-12}
settings_analysis.velocity_translation_threshold = 0.05; % threshold on translational velocity [m/s]
settings_analysis.velocity_rotation_threshold = 0.35; % threshold on rotational velocity [rad/s]

% Parameters in optimal control problems
parameters_OCP.weights.rms_error_orientation = 0.002; % [mm]
parameters_OCP.weights.rms_error_translation = 2*pi/180; % 2 [degrees] converted to [rad]
parameters_OCP.weights.rms_error_force = 0.8; % [N]
parameters_OCP.weights.rms_error_moment = 0.16; % [Nm]
parameters_OCP.weights.L = 0.5; % [m] global scale to weigh the rotational and translational moving frame invariants
parameters_OCP.regul_origin_ASA = 10^(-10); % Used to regulate the origin of the ASA-frame which is used for the initialization of the OCP
parameters_OCP.max_iters = 500; % maximum number of iterations
parameters_OCP.window.window_length = settings_analysis.N;
parameters_OCP.positive_obj_invariant = 0; 
parameters_OCP.positive_mov_invariant = 0;

% Settings - plots
settings_plots.bool_reference_invariants = 1;              % {0,1}
settings_plots.bool_visualize_trials = 0;                  % {0,1}
settings_plots.bool_visualize_reconstruction_errors = 0;   % {0,1}
settings_plots.bool_visualize_summary = 1;                 % {0,1}
settings_plots.bool_paper_plots = 0;                       % {0,1}

%% Load and preprocess data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Check if settings analysis is valid
%check_input(settings_analysis);
%
% Load data from individual trials

path_to_data = 'data/contour_following/reference';
raw_data_reference = load_trials_in_folder(path_to_data);
reference_data = preprocess_contour_reference_data(raw_data_reference{1},settings_analysis);

path_to_data = 'data/contour_following/measurements';
data_exp = load_trials_in_folder(path_to_data);
measurement_data = preprocess_contour_measurement_data(data_exp,settings_analysis);


%% Calculate invariants %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialize OCP by constructing symbolic optimization problem
object = OCP_calculate_screw_invariants_pose(parameters_OCP);

% Initialize results
results.settings_analysis = settings_analysis;
nb_trials = trial_n-trial_0+1; % number of trials
results.trials;

for trial=1:nb_trials
    disp(['analyzing trial ' num2str(trial) '/' num2str(nb_trials) ' ...']);

    % Call class with measurements
    OCP_results = object.calculate_invariants(measured_trajectory);

    % Store results
    results.trials(trial).measured_trajectory = measurements;
    results.trials(trial).invariants = OCP_results.invariants;
    results.trials(trial).reconstructed_trajectory = OCP_results.reconstruction;
    results.trials(trial).moving_frames = OCP_results.moving_frames;
end

disp('analyzing reference invariants:');

% Call class with measurements
OCP_results = object.calculate_invariants(measured_trajectory);

% Store results
results.reference.measured_trajectory = measured_trajectory;
results.reference.invariants = OCP_results.invariants;
results.reference.reconstructed_trajectory = OCP_results.reconstruction;
results.reference.moving_frames = OCP_results.moving_frames;

%% Plotting results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

plot_all_results(results,settings_analysis,settings_plots)
save_results(results,settings_analysis,settings_plots)
