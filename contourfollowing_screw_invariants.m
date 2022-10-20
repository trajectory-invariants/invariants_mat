close all; clear; clc;
addpath(genpath('./implementation/'));

%% %%%%%%%%%%%%%%%%%%%%%%%%% Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%

method = 'new';  % {old, new} (older OCP formulation versus new ocp formulation)

trajectory_type = 'motion'; % {motion, wrench}
viewpoint = 'world'; % {world, body}
referencepoint = 'tracker'; % {tracker, tool_point, force_sensor, middle_contour}

%% Supported combinations in:
% trajectory type + viewpoint +  reference point

% {motion}	+   {world}   +   {tracker}        -> shown in Figure 6a and Figure 9a
% {motion}	+   {world}   +   {tool_point}
% {motion}	+   {body}    +   {middle_contour}
% {wrench}	+   {body}    +   {tracker}        -> shown in Figure 7a and Figure 9c
% {wrench}  +   {body}    +   {tool_point}
% {wrench}  +   {world}   +   {force_sensor}   -> shown in Figure 8 and Figure 9b

%% Parameters
% Parameters for tuning
rms_error_pos = 0.002; % [mm]
rms_error_rot = 2*pi/180; % 2 [degrees] converted to [rad]
rms_error_force = 0.8; % [N]
rms_error_moment = 0.16; % [Nm]
L = 0.5; % [m] global scale to weigh the rotational and translational moving frame invariants

% Parameters for invariants sign
params.positive_obj_invariant = 0;
params.positive_mov_invariant = 0;

% Parameterization of the analysis
parameterization = 'dimless_arclength'; % {time_based, dimless_arclength}

% Parameters of plots
bool_reference_invariants = 1;           % {0,1}
bool_visualize_trials = 0;               % {0,1}
bool_visualize_reconstruction_errors = 0;% {0,1}
bool_visualize_summary = 1;              % {0,1}

% Parameters of input data
N = 101;
trial_0 = 1; % {1-12}
trial_n = 12; % {1-12}

%% %%%%%%%%%%%%%%%%%%%%%%%%% Calculation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%

check_input_screw(trajectory_type,viewpoint,referencepoint);

if strcmp(trajectory_type,'motion')
    bool_motion = 1; % {0,1}
    bool_force = 0; % {0,1}
end
if strcmp(trajectory_type,'wrench')
    bool_motion = 0; % {0,1}
    bool_force = 1; % {0,1}
end

% Load data
[progress,pose,position,rotation,wrench,force,moment,...
    progress_ref,pose_ref,position_ref,rotation_ref,wrench_ref,force_ref,moment_ref] = ...
    contour_preprocess_data(N,viewpoint,parameterization,referencepoint,trial_0,trial_n);
if strcmp(viewpoint,'world')
    wrench_ref = -wrench_ref; force_ref = -force_ref; moment_ref = -moment_ref;
    wrench = -wrench; force = -force; moment = -moment;
end

nb_trials = trial_n-trial_0+1; % number of trials

%% Screw invariants for motion
if bool_motion
    % Parameters optimization problem

    params.weights.rms_error_orientation = rms_error_rot;
    params.weights.rms_error_translation = rms_error_pos;
    params.weights.L = L; % scaling for making positions dimensionless

    params.weights.weight_accuracy_orientation = 1; % weight on measurement fitting term
    params.weights.weight_accuracy_position = 1; % weight on measurement fitting term
    params.weights.weight_regul_deriv_obj = 1e-3; % weight on derivative of object invariants
    params.weights.weight_regul_deriv_mf = 0*1e-10; % weight on derivative of moving frame invariants
    params.weights.weight_regul_abs_mf = 1e-4; % weight on absolute value of moving frame invariants
    params.weights.scale_rotation = 1; % scaling for making rotations comparable to positions
    params.signed_invariants = 1; % if 1, all invariants are allowed to change sign

    params.max_iters = 500;
    params.window.window_length = N;

    % Initialize class by constructing symbolic optimization problem
    if strcmp(method,'old')
        object = OCP_calculate_screw_invariants_pose_old(params);
    else
        object = OCP_calculate_screw_invariants_pose(params);
    end

    % Initialize results
    invars_pose = zeros(N,6,nb_trials);
    recons_pose = zeros(3,4,N,nb_trials);
    T_isa_pose = zeros(3,4,N,nb_trials);

    %% Calculation invariants for motion
    if bool_reference_invariants
        disp('analyzing reference invariants:');

        % Call class with measurements
        h = mean(diff(progress_ref)); % stepsize
        optim_class_result = object.calculate_invariants(pose_ref,h);
        if bool_visualize_trials
            plot_screw_invariants(progress_ref,optim_class_result.invariants,'pose');
            plot_ISA_frames_contour(optim_class_result.ISA_frames,pose_ref,'ref',viewpoint,referencepoint,'motion',parameterization)
            exportgraphics(gcf,['figures/ISA_frames_pose_',viewpoint,'_',referencepoint,'_ref.pdf'],'ContentType','image');
        end

        % Store results
        invars_pose_ref = optim_class_result.invariants;
        recons_pose_ref = optim_class_result.Obj_frames;
    end

    for trial=1:nb_trials
        disp(['analyzing trial ' num2str(trial) '/' num2str(nb_trials) ' ...']);

        % Call class with measurements
        h = mean(diff(progress(:,trial))); % stepsize
        optim_class_result = object.calculate_invariants(pose(:,:,:,trial),h);
        if bool_visualize_trials
            plot_screw_invariants(progress,optim_class_result.invariants,'pose');
            plot_ISA_frames_contour(optim_class_result.ISA_frames,pose(:,:,:,trial),trial+trial_0-1,viewpoint,referencepoint,'motion',parameterization)
            exportgraphics(gcf,['figures/ISA_frames_pose_',viewpoint,'_',referencepoint,'_trial_',num2str(trial),'.pdf'],'ContentType','image');
        end

        % Store results
        invars_pose(:,:,trial) = optim_class_result.invariants;
        recons_pose(:,:,:,trial) = optim_class_result.Obj_frames;
        T_isa_pose(:,:,:,trial) = optim_class_result.ISA_frames;
    end

    %% Plotting results
    if bool_visualize_reconstruction_errors
        figure('Name','measured vs. reconstructed pose','Color',[1 1 1]);
        for trial = trial_0 : trial_n
            if trial == trial_0; tabgroup_pose = uitabgroup; end
            thistab = uitab(tabgroup_pose,'Title',['trial = ',num2str(trial)]); axes('Parent',thistab);
            plot_trajectory_coordinates(progress(:,trial-trial_0+1),[rotm2eul(pose(1:3,1:3,:,trial-trial_0+1),'zyx'),squeeze(pose(1:3,4,:,trial-trial_0+1))'],'pose','measured vs. reconstructed pose',parameterization)
            plot_trajectory_coordinates(progress(:,trial-trial_0+1),[rotm2eul(recons_pose(1:3,1:3,:,trial-trial_0+1),'zyx'),squeeze(recons_pose(1:3,4,:,trial-trial_0+1))'],'pose','measured vs. reconstructed pose',parameterization)
        end
    end

    if bool_visualize_summary
        plot_screw_invariants_contour(progress_ref,invars_pose_ref,progress,invars_pose,'pose');
        exportgraphics(gcf,['figures/screw_invariants_pose_',viewpoint,'_',referencepoint,'.pdf'],'ContentType','vector');
    end
end

%% Screw invariants for force
if bool_force
    % Parameters optimization problem
    params.weights.rms_error_orientation = rms_error_force;
    params.weights.rms_error_translation = rms_error_moment;
    params.weights.L = L; % scaling for making positions dimensionless

    params.weights.weight_accuracy_force = 10; % weight on measurement fitting term
    params.weights.weight_accuracy_torque = 10; % weight on measurement fitting term
    params.weights.weight_regul_deriv_obj = 1e-3; % weight on derivative of object invariants
    params.weights.weight_regul_deriv_mf = 1e-12; % weight on derivative of moving frame invariants
    params.weights.weight_regul_abs_mf = 1e-2; % weight on absolute value of moving frame invariants
    params.weights.scale_rotation = 1; % scaling for making rotations comparable to positions
    params.signed_invariants = 1; % if 1, all invariants are allowed to change sign

    params.window.window_length = N;

    % Initialize class by constructing symbolic optimization problem
    if strcmp(method,'old')
        object = OCP_calculate_screw_invariants_wrench_old(params);
    else
        object = OCP_calculate_screw_invariants_wrench(params);
    end

    % Initialize results
    invars_wrench = zeros(N,6,nb_trials);
    recons_wrench = zeros(N,6,nb_trials);
    T_isa_wrench = zeros(3,4,N,nb_trials);

    %% Calculation invariants for wrench
    if bool_reference_invariants
        disp('analyzing reference invariants:');

        % Call class with measurements
        h = mean(diff(progress_ref)); % stepsize
        optim_class_result = object.calculate_invariants(wrench_ref(:,:),h);
        if bool_visualize_trials
            plot_screw_invariants(progress_ref,optim_class_result.invariants,'pose');
            plot_ISA_frames_contour(optim_class_result.ISA_frames,pose_ref,'ref',viewpoint,referencepoint,'wrench',parameterization)
            exportgraphics(gcf,['figures/ISA_frames_wrench_',viewpoint,'_',referencepoint,'_ref.pdf'],'ContentType','image');
        end

        % Store results
        invars_wrench_ref = optim_class_result.invariants;
        recons_wrench_ref = [optim_class_result.Obj_rotation optim_class_result.Obj_translation];
    end
    for trial=1:nb_trials
        disp(['analyzing trial ' num2str(trial) '/' num2str(nb_trials) ' ...']);

        % Call class with measurements
        h = mean(diff(progress(:,trial))); % stepsize
        optim_class_result = object.calculate_invariants(wrench(:,:,trial),h);
        if bool_visualize_trials
            plot_screw_invariants(progress,optim_class_result.invariants,'pose');
            plot_ISA_frames_contour(optim_class_result.ISA_frames,pose(:,:,:,trial),trial+trial_0-1,viewpoint,referencepoint,'wrench',parameterization)
            exportgraphics(gcf,['figures/ISA_frames_wrench_',viewpoint,'_',referencepoint,'_trial_',num2str(trial+trial_0-1),'.pdf'],'ContentType','image');
        end

        % Store results
        invars_wrench(:,:,trial) = optim_class_result.invariants;
        recons_wrench(:,:,trial) = [optim_class_result.Obj_rotation optim_class_result.Obj_translation];
        T_isa_wrench(:,:,:,trial) = optim_class_result.ISA_frames;
    end

    %% Plotting results
    if bool_visualize_reconstruction_errors
        figure('Name','measured vs. reconstructed wrench','Color',[1 1 1]);
        for trial = trial_0 : trial_n
            if trial == trial_0; tabgroup_wrench = uitabgroup; end
            thistab = uitab(tabgroup_wrench,'Title',['trial = ',num2str(trial)]); axes('Parent',thistab);
            plot_trajectory_coordinates(progress(:,trial-trial_0+1),wrench(:,:,trial-trial_0+1),'wrench','measured vs. reconstructed wrench',parameterization)
            plot_trajectory_coordinates(progress(:,trial-trial_0+1),recons_wrench(:,:,trial-trial_0+1),'wrench','measured vs. reconstructed wrench',parameterization)
        end
    end

    if bool_visualize_summary
        plot_screw_invariants_contour(progress_ref,invars_wrench_ref,progress,invars_wrench,'wrench');
        exportgraphics(gcf,['figures/screw_invariants_wrench_',viewpoint,'_',referencepoint,'.pdf'],'ContentType','vector');
    end
end

