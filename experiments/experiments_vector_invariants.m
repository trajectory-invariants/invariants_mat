close all; clear; clc; restoredefaultpath
addpath(genpath('./implementation/'));
path_to_data_folder = './data/';

%% %%%%%%%%%%%%%%%%%%%%%%%%% Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%

application = 'contour'; % {contour, peg}

trajectory_type = 'position'; % {position, orientation, force, moment}
viewpoint = 'world'; % {world, body}
referencepoint = 'tool_point'; % {tracker, tool_point, force_sensor, middle_contour}
wrenchtype = 'real'; % {real, synthetic}

%% Supported combinations in:
% trajectory type + viewpoint +  reference point

% {orientation}   +   {world}   +   {tracker}           +   {-}         -> shown in Figure 6b
% {orientation}   +   {world}   +   {tool_point}        +   {-}
% {orientation}   +   {bod y}    +   {middle_contour}    +   {-}
% {force}         +   {world}   +   {tracker}           +   {real}
% {force}         +   {world}   +   {tool_point}        +   {real}
% {force}         +   {world}   +   {force_sensor}      +   {real}
% {force}         +   {body}    +   {tracker}           +   {real}      -> shown in Figure 7b
% {force}         +   {body}    +   {tool_point}        +   {real}
% {position}      +   {world}   +   {tracker}           +   {-}         -> shown in Figure 6c
% {position}      +   {world}   +   {tool_point}        +   {-}         -> shown in Figure 6d and Figure 10d
% {position}      +   {body}    +   {tool_point}        +   {-}
% {position}      +   {body}    +   {tracker}           +   {-}
% {position}      +   {body}    +   {middle_contour}    +   {-}
% {moment}        +   {world}   +   {tracker}           +   {real}
% {moment}        +   {world}   +   {tool_point}        +   {real}
% {moment}        +   {world}   +   {force_sensor}      +   {real}
% {moment}        +   {body}    +   {tracker}           +   {real}      -> shown in Figure 7c
% {moment}        +   {body}    +   {tool_point}        +   {real}      -> shown in Figure 7d

%% Parameters
% Parameters for tuning
if strcmp(application,'contour')
    rms_error_pos = 0.002; % [mm]
    rms_error_rot = 2*pi/180; % 2 [degrees] converted to [rad]
    rms_error_force = 0.8; % [N]
    rms_error_moment = 0.16; % [Nm]
elseif strcmp(application,'peg')
    rms_error_pos = 0.002; % [mm]
    rms_error_rot = 2*pi/180; % 2 [degrees] converted to [rad]
    rms_error_force = 0.8; % [N]
    rms_error_moment = 0.16; % [Nm]
end

% Parameters for invariants sign
params.positive_obj_invariant = 0;
params.positive_mov_invariant = 0;
ocp_method = 'new';  % {old, new} (older OCP formulation versus new ocp formulation)

% Parameterization of the analysis
parameterization = 'dimless_arclength'; % {time_based, dimless_arclength}

% Parameters of plots
bool_reference_invariants = 1;              % {0,1}
bool_visualize_trials = 0;                  % {0,1}
bool_visualize_reconstruction_errors = 0;   % {0,1}
bool_visualize_summary = 1;                 % {0,1}
bool_paper_plots = 1;                       % {0,1}

% Parameters of input data
N = 101;
trial_0 = 1; % {1-12}
trial_n = 12; % {1-12}

%% %%%%%%%%%%%%%%%%%%%%%%%%% Calculation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%

check_input_vector(trajectory_type,viewpoint,referencepoint);

[bool_position,bool_orientation,bool_force,bool_moment] = check_vector_analysis_type(trajectory_type);

% Load data
[progress,pose,position,rotation,wrench,force,moment,...
    progress_ref,pose_ref,position_ref,rotation_ref,wrench_ref,force_ref,moment_ref] = ...
    contour_preprocess_data(N,viewpoint,parameterization,referencepoint,trial_0,trial_n,application,wrenchtype,path_to_data_folder);
if strcmp(viewpoint,'world')
    wrench_ref = -wrench_ref; force_ref = -force_ref; moment_ref = -moment_ref;
    wrench = -wrench; force = -force; moment = -moment;
end

nb_trials = trial_n-trial_0+1; % number of trials

%% Vector invariants for position %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if bool_position
    % Parameters of optimization problem
    params.weights.rms_error_traj = rms_error_pos;
    
    params.weights.weight_accuracy = 1; % weight on measurement fitting term
    params.weights.weight_regul_deriv_obj = 1e-3; % weight on derivative of object invariants
    params.weights.weight_regul_deriv_mf = 1e-6; % weight on derivative of moving frame invariants
    params.weights.weight_regul_abs_mf = 1e-5; % weight on absolute value of moving frame invariants
    params.weights.scale_rotation = 1; % scaling for making rotations comparable to positions
    params.signed_invariants = 1; % if 1, all invariants are allowed to change sign
    
    params.window.window_length = N;
    
    % Initialize class by constructing symbolic optimization problem
    if strcmp(ocp_method,'old')
        object = OCP_calculate_vector_invariants_position_old(params);
    else
        object = OCP_calculate_vector_invariants_position(params);
    end
    
    % Initialize results
    invars_position = zeros(N,3,nb_trials);
    recons_position = zeros(N,3,nb_trials);
    R_FS_position = zeros(3,3,N,nb_trials);
    
    % Calculate invariants for position %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if bool_reference_invariants
        disp('analyzing reference invariants:');
        
        % Call class with measurements
        h = mean(diff(progress_ref)); % stepsize
        optim_class_result = object.calculate_invariants(position_ref,h);
        
        % Store results
        invars_position_ref = optim_class_result.invariants;
        recons_position_ref = optim_class_result.Obj_location;
        R_FS_position_ref = optim_class_result.FS_frames;
    else
        invars_position_ref = NaN; recons_position_ref = NaN; T_isa_position_ref = NaN;
    end
    
    for trial=1:nb_trials
        
        disp(['analyzing trial ' num2str(trial) '/' num2str(nb_trials) ' ...']);
        
        % Call class with measurements
        h = mean(diff(progress(:,trial))); % stepsize
        optim_class_result = object.calculate_invariants(position(:,:,trial),h);
        
        % Store results
        invars_position(:,:,trial) = optim_class_result.invariants;
        recons_position(:,:,trial) = optim_class_result.Obj_location;
        R_FS_position(:,:,:,trial) = optim_class_result.FS_frames;
    end
    
    % Plotting results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    plot_all_results_vector('position',bool_reference_invariants,bool_visualize_trials,bool_visualize_reconstruction_errors,bool_visualize_summary,bool_paper_plots,...
        progress_ref,invars_position_ref,R_FS_position_ref,pose_ref,viewpoint,referencepoint,parameterization,application,position_ref,...
        recons_position_ref,nb_trials,progress,invars_position,R_FS_position,trial_0,position,recons_position,pose,trajectory_type,wrenchtype,path_to_data_folder)
end

%% Vector invariants for orientation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if bool_orientation
    % Parameters of optimization problem
    params.weights.rms_error_traj = rms_error_rot;
    
    params.weights.weight_accuracy = 10; % weight on measurement fitting term
    params.weights.weight_regul_deriv_obj = 1e-3; % weight on derivative of object invariants
    params.weights.weight_regul_deriv_mf = 1e-10; % weight on derivative of moving frame invariants
    params.weights.weight_regul_abs_mf = 1e-4; % weight on absolute value of moving frame invariants
    params.weights.scale_rotation = 1; % scaling for making rotations comparable to positions
    params.signed_invariants = 1; % if 1, all invariants are allowed to change sign
    
    params.window.window_length = N;
    
    % Initialize class by constructing symbolic optimization problem
    if strcmp(ocp_method,'old')
        object = OCP_calculate_vector_invariants_rotation_old(params);
    else
        object = OCP_calculate_vector_invariants_rotation(params);
    end
    
    % Initialize results
    invars_rotation = zeros(N,3,nb_trials);
    recons_rotation = zeros(3,3,N,nb_trials);
    R_FS_rotation = zeros(3,3,N,nb_trials);
    
    % Calculate invariants for orientation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if bool_reference_invariants
        disp('analyzing reference invariants:');
        
        % Call class with measurements
        h = mean(diff(progress_ref)); % stepsize
        optim_class_result = object.calculate_invariants(rotation_ref,h);
        
        % Store results
        invars_rotation_ref = optim_class_result.invariants;
        recons_rotation_ref = optim_class_result.Obj_frames;
        R_FS_rotation_ref = optim_class_result.FS_frames;
    else
        invars_rotation_ref = NaN; recons_rotation_ref = NaN; T_isa_rotation_ref = NaN;
    end
    
    for trial=1:nb_trials
        disp(['analyzing trial ' num2str(trial) '/' num2str(nb_trials) ':']);
        
        % Call class with measurements
        h = mean(diff(progress(:,trial))); % stepsize
        optim_class_result = object.calculate_invariants(rotation(:,:,:,trial),h);
        
        % Store results
        invars_rotation(:,:,trial) = optim_class_result.invariants;
        recons_rotation(:,:,:,trial) = optim_class_result.Obj_frames;
        R_FS_rotation(:,:,:,trial) = optim_class_result.FS_frames;
    end
    
    % Plotting results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    plot_all_results_vector('orientation',bool_reference_invariants,bool_visualize_trials,bool_visualize_reconstruction_errors,bool_visualize_summary,bool_paper_plots,...
        progress_ref,invars_rotation_ref,R_FS_rotation_ref,pose_ref,viewpoint,referencepoint,parameterization,application,rotation_ref,...
        recons_rotation_ref,nb_trials,progress,invars_rotation,R_FS_rotation,trial_0,rotation,recons_rotation,pose,trajectory_type,wrenchtype,path_to_data_folder)
end

%% Vector invariants for force %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if bool_force
    % Parameters of optimization problem
    params.weights.rms_error_traj = rms_error_force;
    
    params.weights.weight_accuracy = 1; % weight on measurement fitting term
    params.weights.weight_regul_deriv_obj = 1e-0; % weight on derivative of object invariants
    params.weights.weight_regul_deriv_mf = 1e-4; % weight on derivative of moving frame invariants
    params.weights.weight_regul_abs_mf = 0*1e-0; % weight on absolute value of moving frame invariants
    params.weights.scale_rotation = 1; % scaling for making rotations comparable to positions
    params.signed_invariants = 1; % if 1, all invariants are allowed to change sign
    
    params.window.window_length = N;
    
    % Initialize class by constructing symbolic optimization problem
    if strcmp(ocp_method,'old')
        object = OCP_calculate_vector_invariants_vector_old(params);
    else
        object = OCP_calculate_vector_invariants_vector(params);
    end
    
    % Initialize results
    invars_force = zeros(N,3,nb_trials);
    recons_force = zeros(N,3,nb_trials);
    R_FS_force = zeros(3,3,N,nb_trials);
    
    % Calculate invariants for force %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if bool_reference_invariants
        disp('analyzing reference invariants:');
        
        % Call class with measurements
        h = mean(diff(progress_ref)); % stepsize
        optim_class_result = object.calculate_invariants(force_ref,h);
        
        % Store results
        invars_force_ref = optim_class_result.invariants;
        recons_force_ref = optim_class_result.Obj_trajectory;
        R_FS_force_ref = optim_class_result.FS_frames;
    else
        invars_force_ref = NaN; recons_force_ref = NaN; T_isa_force_ref = NaN;
    end
    
    for trial=1:nb_trials
        disp(['analyzing trial ' num2str(trial) '/' num2str(nb_trials) ':']);
        
        % Call class with measurements
        h = mean(diff(progress(:,trial))); % stepsize
        optim_class_result = object.calculate_invariants(force(:,:,trial),h);
        
        % Store results
        invars_force(:,:,trial) = optim_class_result.invariants(:,1:3);
        recons_force(:,:,trial) = optim_class_result.Obj_trajectory;
        R_FS_force(:,:,:,trial) = optim_class_result.FS_frames;
    end
    
    % Plotting results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    plot_all_results_vector('force',bool_reference_invariants,bool_visualize_trials,bool_visualize_reconstruction_errors,bool_visualize_summary,bool_paper_plots,...
        progress_ref,invars_force_ref,R_FS_force_ref,pose_ref,viewpoint,referencepoint,parameterization,application,force_ref,...
        recons_force_ref,nb_trials,progress,invars_force,R_FS_force,trial_0,force,recons_force,pose,trajectory_type,wrenchtype,path_to_data_folder)
end

%% Vector invariants for moment %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if bool_moment
    % Parameters of optimization problem
    params.weights.rms_error_traj = rms_error_moment;
    
    params.weights.weight_accuracy = 10; % weight on measurement fitting term
    params.weights.weight_regul_deriv_obj = 1e-0; % weight on derivative of object invariants
    params.weights.weight_regul_deriv_mf = 1e-3; % weight on derivative of moving frame invariants
    params.weights.weight_regul_abs_mf = 1e-2; % weight on absolute value of moving frame invariants
    params.weights.scale_rotation = 1; % scaling for making rotations comparable to positions
    params.signed_invariants = 1; % if 1, all invariants are allowed to change sign
    
    params.window.window_length = N;
    
    % Initialize class by constructing symbolic optimization problem
    object = OCP_calculate_vector_invariants_vector(params);
    
    % Initialize results
    invars_moment = zeros(N,3,nb_trials);
    recons_moment = zeros(N,3,nb_trials);
    R_FS_moment = zeros(3,3,N,nb_trials);
    
    % Calculate invariants for moment %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if bool_reference_invariants
        disp('analyzing reference invariants:');
        
        % Call class with measurements
        h = mean(diff(progress_ref)); % stepsize
        optim_class_result = object.calculate_invariants(moment_ref,h);
        
        % Store results
        invars_moment_ref = optim_class_result.invariants;
        recons_moment_ref = optim_class_result.Obj_trajectory;
        R_FS_moment_ref = optim_class_result.FS_frames;
    else
        invars_moment_ref = NaN; recons_moment_ref = NaN; T_isa_moment_ref = NaN;
    end
    
    for trial=1:nb_trials
        disp(['analyzing trial ' num2str(trial) '/' num2str(nb_trials) ':']);
        
        % Call class with measurements
        h = mean(diff(progress(:,trial))); % stepsize
        optim_class_result = object.calculate_invariants(moment(:,:,trial),h);
        
        % Store results
        invars_moment(:,:,trial) = optim_class_result.invariants(:,1:3);
        recons_moment(:,:,trial) = optim_class_result.Obj_trajectory;
        R_FS_moment(:,:,:,trial) = optim_class_result.FS_frames;
    end
    
    % Plotting results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    plot_all_results_vector('moment',bool_reference_invariants,bool_visualize_trials,bool_visualize_reconstruction_errors,bool_visualize_summary,bool_paper_plots,...
        progress_ref,invars_moment_ref,R_FS_moment_ref,pose_ref,viewpoint,referencepoint,parameterization,application,moment_ref,...
        recons_moment_ref,nb_trials,progress,invars_moment,R_FS_moment,trial_0,moment,recons_moment,pose,trajectory_type,wrenchtype,path_to_data_folder)
end
