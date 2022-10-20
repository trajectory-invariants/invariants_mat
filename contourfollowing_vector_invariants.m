close all; clear; clc;
addpath(genpath('./implementation/'));

%% %%%%%%%%%%%%%%%%%%%%%%%%% Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%

method = 'new';  % {old, new} (older OCP formulation versus new ocp formulation)

trajectory_type = 'orientation'; % {position, orientation, force, moment}
viewpoint = 'world'; % {world, body}
referencepoint = 'tracker'; % {tracker, tool_point, force_sensor, middle_contour}

%% Supported combinations in:
% trajectory type + viewpoint +  reference point

% {orientation}   +   {world}   +   {tracker}           -> shown in Figure 6b
% {orientation}   +   {world}   +   {tool_point}
% {orientation}   +   {body}    +   {tracker}
% {orientation}   +   {body}    +   {tool_point}
% {orientation}   +   {body}    +   {middle_contour}
% {force}         +   {world}   +   {tracker}
% {force}         +   {world}   +   {tool_point}
% {force}         +   {world}   +   {force_sensor}
% {force}         +   {body}    +   {tracker}           -> shown in Figure 7b
% {force}         +   {body}    +   {tool_point}
% {position}      +   {world}   +   {tracker}           -> shown in Figure 6c
% {position}      +   {world}   +   {tool_point}        -> shown in Figure 6d and Figure 9d
% {position}      +   {body}    +   {tool_point}
% {position}      +   {body}    +   {tracker}
% {position}      +   {body}    +   {middle_contour}
% {moment}        +   {world}   +   {tracker}
% {moment}        +   {world}   +   {tool_point}
% {moment}        +   {world}   +   {force_sensor}
% {moment}        +   {body}    +   {tracker}           -> shown in Figure 7c
% {moment}        +   {body}    +   {tool_point}        -> shown in Figure 7d

%% Parameters
% Parameters for tuning
rms_error_pos = 0.002; % [mm]
rms_error_rot = 2*pi/180; % 2 [degrees] converted to [rad]
rms_error_force = 0.8; % [N]
rms_error_moment = 0.16; % [Nm]

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

%% %%%%%%%%%%%%%%%%%%%%%%%%% Calculation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%

check_input_vector(trajectory_type,viewpoint,referencepoint);

if strcmp(trajectory_type,'position')
    bool_position = 1; % {0,1}
    bool_orientation = 0; % {0,1}
    bool_force = 0; % {0,1}
    bool_moment = 0; % {0,1}
end
if strcmp(trajectory_type,'orientation')
    bool_position = 0; % {0,1}
    bool_orientation = 1; % {0,1}
    bool_force = 0; % {0,1}
    bool_moment = 0; % {0,1}
end
if strcmp(trajectory_type,'force')
    bool_position = 0; % {0,1}
    bool_orientation = 0; % {0,1}
    bool_force = 1; % {0,1}
    bool_moment = 0; % {0,1}
end
if strcmp(trajectory_type,'moment')
    bool_position = 0; % {0,1}
    bool_orientation = 0; % {0,1}
    bool_force = 0; % {0,1}
    bool_moment = 1; % {0,1}
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

%% Vector invariants for position
if bool_position
    % Parameters optimization problem
    params.weights.rms_error_traj = rms_error_pos;

    params.weights.weight_accuracy = 1; % weight on measurement fitting term
    params.weights.weight_regul_deriv_obj = 1e-3; % weight on derivative of object invariants
    params.weights.weight_regul_deriv_mf = 1e-6; % weight on derivative of moving frame invariants
    params.weights.weight_regul_abs_mf = 1e-5; % weight on absolute value of moving frame invariants
    params.weights.scale_rotation = 1; % scaling for making rotations comparable to positions
    params.signed_invariants = 1; % if 1, all invariants are allowed to change sign

    params.window.window_length = N;

    % Initialize class by constructing symbolic optimization problem
    if strcmp(method,'old')
        object = OCP_calculate_vector_invariants_position_old(params);
    else
        object = OCP_calculate_vector_invariants_position(params);
    end

    % Initialize results
    invars_position = zeros(N,3,nb_trials);
    recons_position = zeros(N,3,nb_trials);
    R_FS_position = zeros(3,3,N,nb_trials);

    %% Calculation invariants for motion
    if bool_reference_invariants
        disp('analyzing reference invariants:');

        % Call class with measurements
        h = mean(diff(progress_ref)); % stepsize
        optim_class_result = object.calculate_invariants(position_ref,h);
        if bool_visualize_trials
            plot_vector_invariants(progress_ref,optim_class_result.invariants,'position')
            plot_FS_frames_contour(optim_class_result.FS_frames,optim_class_result.invariants(:,1),...
                pose_ref,'ref',viewpoint,referencepoint,'position',parameterization);
            exportgraphics(gcf,['figures/FS_frames_position_',viewpoint,'_',referencepoint,'_ref.pdf'],'ContentType','image');
        end

        % Store results
        invars_position_ref = optim_class_result.invariants;
        recons_position_ref = optim_class_result.Obj_location;
    end

    for trial=1:nb_trials

        disp(['analyzing trial ' num2str(trial) '/' num2str(nb_trials) ' ...']);

        % Call class with measurements
        h = mean(diff(progress(:,trial))); % stepsize
        optim_class_result = object.calculate_invariants(position(:,:,trial),h);

        if bool_visualize_trials
            plot_vector_invariants(progress(:,trial),optim_class_result.invariants,'position')
            plot_FS_frames_contour(optim_class_result.FS_frames,optim_class_result.invariants(:,1),...
                pose(:,:,:,trial),trial+trial_0-1,viewpoint,referencepoint,'position',parameterization);
            exportgraphics(gcf,['figures/FS_frames_position_',viewpoint,'_',referencepoint,'_trial_',num2str(trial+trial_0-1),'.pdf'],'ContentType','image');
        end

        % Store results
        invars_position(:,:,trial) = optim_class_result.invariants;
        recons_position(:,:,trial) = optim_class_result.Obj_location;
        R_FS_position(:,:,:,trial) = optim_class_result.FS_frames;

    end

    %% Plotting results
    if bool_visualize_reconstruction_errors
        figure('Name','measured vs. reconstructed position','Color',[1 1 1]);
        for trial = trial_0 : trial_n
            if trial == trial_0; tabgroup_position = uitabgroup; end
            thistab = uitab(tabgroup_position,'Title',['trial = ',num2str(trial)]); axes('Parent',thistab);
            plot_trajectory_coordinates(progress(:,trial-trial_0+1),position(:,:,trial-trial_0+1),'position','measured vs. reconstructed position',parameterization)
            plot_trajectory_coordinates(progress(:,trial-trial_0+1),recons_position(:,:,trial-trial_0+1),'position','measured vs. reconstructed position',parameterization)
        end
    end

    if bool_visualize_summary
        plot_vector_invariants_contour(progress_ref,invars_position_ref,progress,invars_position,'position');
        exportgraphics(gcf,['figures/vector_invariants_position_',viewpoint,'_',referencepoint,'.pdf'],'ContentType','vector');
    end
end

%% Vector invariants for orientation
if bool_orientation
    % Parameters optimization problem
    params.weights.rms_error_traj = rms_error_rot;

    params.weights.weight_accuracy = 10; % weight on measurement fitting term
    params.weights.weight_regul_deriv_obj = 1e-3; % weight on derivative of object invariants
    params.weights.weight_regul_deriv_mf = 1e-10; % weight on derivative of moving frame invariants
    params.weights.weight_regul_abs_mf = 1e-4; % weight on absolute value of moving frame invariants
    params.weights.scale_rotation = 1; % scaling for making rotations comparable to positions
    params.signed_invariants = 1; % if 1, all invariants are allowed to change sign

    params.window.window_length = N;

    % Initialize class by constructing symbolic optimization problem
    if strcmp(method,'old')
        object = OCP_calculate_vector_invariants_rotation_old(params);
    else
        object = OCP_calculate_vector_invariants_rotation(params);
    end

    % Initialize results
    invars_rotation = zeros(N,3,nb_trials);
    recons_rotation = zeros(3,3,N,nb_trials);
    R_FS_rotation = zeros(3,3,N,nb_trials);

    %% Calculation invariants for motion
    if bool_reference_invariants
        disp('analyzing reference invariants:');

        % Call class with measurements
        h = mean(diff(progress_ref)); % stepsize
        optim_class_result = object.calculate_invariants(rotation_ref,h);
        %figure; hold on; plot(squeeze(pose_ref(1:3,2,:))','b'); plot(squeeze(optim_class_result.Obj_frames(1:3,2,:))','r');
        if bool_visualize_trials
            plot_vector_invariants(progress_ref,optim_class_result.invariants,'rotation')
            plot_FS_frames_contour(optim_class_result.FS_frames,optim_class_result.invariants(:,1),...
                pose_ref,'ref',viewpoint,referencepoint,'rotation',parameterization);
            exportgraphics(gcf,['figures/FS_frames_orientation_',viewpoint,'_',referencepoint,'_ref.pdf'],'ContentType','image');
        end

        % Store results
        invars_rotation_ref = optim_class_result.invariants;
        recons_rotation_ref = optim_class_result.Obj_frames;
    end

    for trial=1:nb_trials
        disp(['analyzing trial ' num2str(trial) '/' num2str(nb_trials) ':']);

        % Call class with measurements
        h = mean(diff(progress(:,trial))); % stepsize
        optim_class_result = object.calculate_invariants(rotation(:,:,:,trial),h);
        % figure; hold on; plot(squeeze(pose(1:3,3,:,trial))','b'); plot(squeeze(optim_class_result.Obj_location(1:3,3,:))','r');
        if bool_visualize_trials
            plot_vector_invariants(progress(:,trial),optim_class_result.invariants,'rotation')
            plot_FS_frames_contour(optim_class_result.FS_frames,optim_class_result.invariants(:,1),...
                pose(:,:,:,trial),trial+trial_0-1,viewpoint,referencepoint,'rotation',parameterization);
            exportgraphics(gcf,['figures/FS_frames_orientation_',viewpoint,'_',referencepoint,'_trial_',num2str(trial),'.pdf'],'ContentType','image');
        end

        % Store results
        invars_rotation(:,:,trial) = optim_class_result.invariants;
        recons_rotation(:,:,:,trial) = optim_class_result.Obj_frames;
        R_FS_rotation(:,:,:,trial) = optim_class_result.FS_frames;
    end

    %% Plotting results
    if bool_visualize_reconstruction_errors
        figure('Name','measured vs. reconstructed rotation','Color',[1 1 1]);
        for trial = trial_0 : trial_n
            if trial == trial_0; tabgroup_rotation = uitabgroup; end
            thistab = uitab(tabgroup_rotation,'Title',['trial = ',num2str(trial)]); axes('Parent',thistab);
            plot_trajectory_coordinates(progress(:,trial-trial_0+1),rotm2eul(rotation(:,:,:,trial-trial_0+1)),'rotation','measured vs. reconstructed rotation',parameterization)
            plot_trajectory_coordinates(progress(:,trial-trial_0+1),rotm2eul(recons_rotation(:,:,:,trial-trial_0+1)),'rotation','measured vs. reconstructed rotation',parameterization)
        end
    end

    if bool_visualize_summary
        plot_vector_invariants_contour(progress_ref,invars_rotation_ref,progress,invars_rotation,'rotation');
        exportgraphics(gcf,['figures/vector_invariants_rotation_',viewpoint,'_',referencepoint,'.pdf'],'ContentType','vector');
    end
end

%% Vector invariants for force
if bool_force
    % Parameters optimization problem
    params.weights.rms_error_traj = rms_error_force;

    params.weights.weight_accuracy = 1; % weight on measurement fitting term
    params.weights.weight_regul_deriv_obj = 1e-0; % weight on derivative of object invariants
    params.weights.weight_regul_deriv_mf = 1e-4; % weight on derivative of moving frame invariants
    params.weights.weight_regul_abs_mf = 0*1e-0; % weight on absolute value of moving frame invariants
    params.weights.scale_rotation = 1; % scaling for making rotations comparable to positions
    params.signed_invariants = 1; % if 1, all invariants are allowed to change sign

    params.window.window_length = N;

    % Initialize class by constructing symbolic optimization problem
    if strcmp(method,'old')
        object = OCP_calculate_vector_invariants_vector_old(params);
    else
        object = OCP_calculate_vector_invariants_vector(params);
    end

    % Initialize results
    invars_force = zeros(N,3,nb_trials);
    recons_force = zeros(N,3,nb_trials);
    R_FS_force = zeros(3,3,N,nb_trials);

    %% Calculation invariants
    if bool_reference_invariants
        disp('analyzing reference invariants:');

        % Call class with measurements
        h = mean(diff(progress_ref)); % stepsize
        optim_class_result = object.calculate_invariants(force_ref,h);
        if bool_visualize_trials
            plot_vector_invariants(progress_ref,optim_class_result.invariants,'force')
            plot_FS_frames_contour(optim_class_result.FS_frames,optim_class_result.invariants(:,1),...
                pose_ref,'ref',viewpoint,referencepoint,'force',parameterization);
            exportgraphics(gcf,['figures/FS_frames_force_',viewpoint,'_',referencepoint,'_ref.pdf'],'ContentType','image');
        end

        % Store results
        invars_force_ref = optim_class_result.invariants;
        recons_force_ref = optim_class_result.Obj_trajectory;
    end

    for trial=1:nb_trials
        disp(['analyzing trial ' num2str(trial) '/' num2str(nb_trials) ':']);

        % Call class with measurements
        h = mean(diff(progress(:,trial))); % stepsize
        optim_class_result = object.calculate_invariants(force(:,:,trial),h);
        if bool_visualize_trials
            plot_vector_invariants(progress(:,trial),optim_class_result.invariants,'force')
            plot_FS_frames_contour(optim_class_result.FS_frames,optim_class_result.invariants(:,1),...
                pose(:,:,:,trial),trial+trial_0-1,viewpoint,referencepoint,'force',parameterization);
            exportgraphics(gcf,['figures/FS_frames_force_',viewpoint,'_',referencepoint,'_trial_',num2str(trial),'.pdf'],'ContentType','image');
        end

        % Store results
        invars_force(:,:,trial) = optim_class_result.invariants(:,1:3);
        recons_force(:,:,trial) = optim_class_result.Obj_trajectory;
        R_FS_force(:,:,:,trial) = optim_class_result.FS_frames;
    end

    %% Plotting results
    if bool_visualize_reconstruction_errors
        figure('Name','measured vs. reconstructed force','Color',[1 1 1]);
        for trial = trial_0 : trial_n
            if trial == trial_0; tabgroup_force = uitabgroup; end
            thistab = uitab(tabgroup_force,'Title',['trial = ',num2str(trial)]); axes('Parent',thistab);
            plot_trajectory_coordinates(progress(:,trial-trial_0+1),force(:,:,trial-trial_0+1),'force','measured vs. reconstructed force',parameterization)
            plot_trajectory_coordinates(progress(:,trial-trial_0+1),recons_force(:,:,trial-trial_0+1),'force','measured vs. reconstructed force',parameterization)
        end
    end

    if bool_visualize_summary
        plot_vector_invariants_contour(progress_ref,invars_force_ref,progress,invars_force,'force');
        exportgraphics(gcf,['figures/vector_invariants_force_',viewpoint,'_',referencepoint,'.pdf'],'ContentType','vector');
    end
end

%% Vector invariants for moment
if bool_moment
    % Parameters optimization problem
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

    %% Calculation invariants
    if bool_reference_invariants
        disp('analyzing reference invariants:');

        % Call class with measurements
        h = mean(diff(progress_ref)); % stepsize
        optim_class_result = object.calculate_invariants(moment_ref,h);

        if bool_visualize_trials
            plot_vector_invariants(progress_ref,optim_class_result.invariants,'moment');
            plot_FS_frames_contour(optim_class_result.FS_frames,optim_class_result.invariants(:,1),...
                pose_ref,'ref',viewpoint,referencepoint,'moment',parameterization);
            exportgraphics(gcf,['figures/FS_frames_moment_',viewpoint,'_',referencepoint,'_ref.pdf'],'ContentType','image');
        end

        % Store results
        invars_moment_ref = optim_class_result.invariants;
        recons_moment_ref = optim_class_result.Obj_trajectory;
    end
    for trial=1:nb_trials
        disp(['analyzing trial ' num2str(trial) '/' num2str(nb_trials) ':']);

        % Call class with measurements
        h = mean(diff(progress(:,trial))); % stepsize
        optim_class_result = object.calculate_invariants(moment(:,:,trial),h);

        if bool_visualize_trials
            plot_vector_invariants(progress(:,trial),optim_class_result.invariants,'moment');
            plot_FS_frames_contour(optim_class_result.FS_frames,optim_class_result.invariants(:,1),...
                pose(:,:,:,trial),trial+trial_0-1,viewpoint,referencepoint,'moment',parameterization);
            exportgraphics(gcf,['figures/FS_frames_moment_',viewpoint,'_',referencepoint,'_trial_',num2str(trial),'.pdf'],'ContentType','image');
        end

        % Store results
        invars_moment(:,:,trial) = optim_class_result.invariants(:,1:3);
        recons_moment(:,:,trial) = optim_class_result.Obj_trajectory;
        R_FS_moment(:,:,:,trial) = optim_class_result.FS_frames;
    end
    %% Plotting results
    if bool_visualize_reconstruction_errors
        figure('Name','measured vs. reconstructed moment','Color',[1 1 1]);
        for trial = trial_0 : trial_n
            if trial == trial_0; tabgroup_moment = uitabgroup; end
            thistab = uitab(tabgroup_moment,'Title',['trial = ',num2str(trial)]); axes('Parent',thistab);
            plot_trajectory_coordinates(progress(:,trial-trial_0+1),moment(:,:,trial-trial_0+1),'moment','measured vs. reconstructed moment',parameterization)
            plot_trajectory_coordinates(progress(:,trial-trial_0+1),recons_moment(:,:,trial-trial_0+1),'moment','measured vs. reconstructed moment',parameterization)
        end
    end

    if bool_visualize_summary
        plot_vector_invariants_contour(progress_ref,invars_moment_ref,progress,invars_moment,'moment');
        exportgraphics(gcf,['figures/vector_invariants_moment_',viewpoint,'_',referencepoint,'.pdf'],'ContentType','vector');
    end
end
