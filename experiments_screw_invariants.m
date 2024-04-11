close all; clear; clc; %restoredefaultpath
addpath(genpath('./implementation/'));
path_to_data_folder = './data/';

%% %%%%%%%%%%%%%%%%%%%%%%%%% Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%

application = 'contour'; % {contour, peg}
trajectory_type = 'wrench'; % {motion, wrench}
viewpoint = 'world'; % {world, body}
referencepoint = 'force_sensor'; % {tracker, tool_point, force_sensor, middle_contour}
wrenchtype = 'real'; % {real, synthetic}

% Supported combinations in:
% trajectory type + viewpoint +  reference point + wrenchtype

% {motion}	+   {world}   +   {tracker}         +   {real}      -> used for Figures 6a, 10a, 13a, 14a
% {motion}	+   {world}   +   {tool_point}      +   {real}
% {motion}	+   {body}    +   {middle_contour}  +   {real}
% {wrench}	+   {body}    +   {tracker}         +   {real}      -> shown in Figure 7a and Figure 10b
% {wrench}  +   {body}    +   {tool_point}      +   {real}
% {wrench}  +   {world}   +   {force_sensor}    +   {real}      -> shown in Figures 8, 10c, 13b, 14b
% {wrench}  +   {world}   +   {force_sensor}    +   {synthetic} -> shown in Figure 9

check_input_screw(trajectory_type,viewpoint,referencepoint);

%% Parameters
% Parameters for tuning
if strcmp(application,'contour')
    rms_error_pos = 0.002; % [mm]
    rms_error_rot = 2*pi/180; % 2 [degrees] converted to [rad]
    rms_error_force = 0.8; % [N]
    rms_error_moment = 0.16; % [Nm]
    L = 0.5; % [m] global scale to weigh the rotational and translational moving frame invariants
elseif strcmp(application,'peg')
    rms_error_pos = 0.002; % [mm]
    rms_error_rot = 2*pi/180; % 2 [degrees] converted to [rad]
    rms_error_force = 0.3; % [N]
    rms_error_moment = 0.1; % [Nm]
    L = 0.5; % [m] global scale to weigh the rotational and translational moving frame invariants
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
bool_paper_plots = 0;                       % {0,1}

% Parameters of input data
N = 101;
trial_0 = 5; % {1-12}
trial_n = 5; % {1-12}

%% Load data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Load data from individual trials
[progress,pose,position,rotation,wrench,force,moment,...
    progress_ref,pose_ref,position_ref,rotation_ref,wrench_ref,force_ref,moment_ref] = ...
    contour_preprocess_data(N,viewpoint,parameterization,referencepoint,trial_0,trial_n,application,wrenchtype,path_to_data_folder);

% Invert sign of force when viewed in the world (reaction forces)
if strcmp(viewpoint,'world')
    wrench_ref = -wrench_ref; force_ref = -force_ref; moment_ref = -moment_ref;
    wrench = -wrench; force = -force; moment = -moment;
end

nb_trials = trial_n-trial_0+1; % number of trials
[bool_motion,bool_force] = check_screw_analysis_type(trajectory_type);

%% Screw invariants for motion %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if strcmp(application,'contour')
    params.regul_origin_ASA = 10^(-10); % Used to regulate the origin of the ASA-frame which is used for the initialization of the OCP
elseif strcmp(application,'peg')
    params.regul_origin_ASA = 10^(-6);  % Used to regulate the origin of the ASA-frame which is used for the initialization of the OCP
end

counter = 1;
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
    if strcmp(ocp_method,'old')
        object = OCP_calculate_screw_invariants_pose_old(params);
    else
        object = OCP_calculate_screw_invariants_pose(params);
    end
    
    % Initialize results
    invars_pose = zeros(N,6,nb_trials);
    recons_pose = zeros(3,4,N,nb_trials);
    T_isa_pose = zeros(3,4,N,nb_trials);
    
    % Calculation invariants for motion %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if bool_reference_invariants
        disp('analyzing reference invariants:');
        
        % Call class with measurements
        h = mean(diff(progress_ref)); % stepsize
        optim_class_result = object.calculate_invariants(pose_ref,h,params);
        
        % Store results
        invars_pose_ref = optim_class_result.invariants;
        recons_pose_ref = optim_class_result.Obj_frames;
        ISA_frames_pose_ref = optim_class_result.ISA_frames;

        T_isa_pose_ref = optim_class_result.ISA_frames;
    else
        invars_pose_ref = NaN; recons_pose_ref = NaN; T_isa_pose_ref = NaN;
    end
    
    for trial=1:nb_trials
        disp(['analyzing trial ' num2str(trial) '/' num2str(nb_trials) ' ...']);
        
        % Call class with measurements
        h = mean(diff(progress(:,trial))); % stepsize
        optim_class_result = object.calculate_invariants(pose(:,:,:,trial),h,params);
        
        % Store results
        invars_pose(:,:,trial) = optim_class_result.invariants;
        recons_pose(:,:,:,trial) = optim_class_result.Obj_frames;
        T_isa_pose(:,:,:,trial) = optim_class_result.ISA_frames;
        
        result(counter).invars_pose = invars_pose(:,:,trial);
        result(counter).recons_pose = recons_pose(:,:,:,trial);
        result(counter).ISA_frames_pose = T_isa_pose(:,:,:,trial);
        result(counter).method = ocp_method;
        result(counter).trajectory_type = trajectory_type;
        result(counter).viewpoint = viewpoint; 
        result(counter).referencepoint = referencepoint; % {tracker, tool_point, force_sensor, middle_contour}
        result(counter).trial = trial;
        result(counter).measured_pose = pose(:,:,:,trial);
        result(counter).h = h;
        counter = counter + 1;
                
    end
    
    % Plotting results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    plot_all_results_screw('pose',bool_reference_invariants,bool_visualize_trials,bool_visualize_reconstruction_errors,bool_visualize_summary,bool_paper_plots,...
        progress_ref,invars_pose_ref,T_isa_pose_ref,pose_ref,viewpoint,referencepoint,parameterization,application,...
        recons_pose_ref,nb_trials,progress,invars_pose,T_isa_pose,pose,trial_0,recons_pose,pose,pose_ref,trajectory_type,wrenchtype,path_to_data_folder)
end

%% Screw invariants for force %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
counter = 1;
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
    if strcmp(ocp_method,'old')
        object = OCP_calculate_screw_invariants_wrench_old(params);
    else
        object = OCP_calculate_screw_invariants_wrench(params);
    end
    
    % Initialize results
    invars_wrench = zeros(N,6,nb_trials);
    recons_wrench = zeros(N,6,nb_trials);
    T_isa_wrench = zeros(3,4,N,nb_trials);
    
    % Calculation invariants for wrench %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if bool_reference_invariants
        disp('analyzing reference invariants:');
        
        % Call class with measurements
        h = mean(diff(progress_ref)); % stepsize
        optim_class_result = object.calculate_invariants(wrench_ref(:,:),h,params);
        
        % Store results
        invars_wrench_ref = optim_class_result.invariants;
        recons_wrench_ref = [optim_class_result.Obj_rotation optim_class_result.Obj_translation];
        T_isa_wrench_ref = optim_class_result.ISA_frames;
        
        result(counter).invars_wrench = invars_wrench_ref;
        result(counter).recons_wrench = recons_wrench_ref;
        result(counter).ISA_frames_wrench = T_isa_wrench_ref;
        result(counter).method = ocp_method;
        result(counter).trajectory_type = trajectory_type;
        result(counter).viewpoint = viewpoint; 
        result(counter).referencepoint = referencepoint; % {tracker, tool_point, force_sensor, middle_contour}
        result(counter).trial = 'ref';
        result(counter).measured_wrench = wrench_ref;
        result(counter).h = h;
        counter = counter + 1;
    else
        invars_wrench_ref = NaN; recons_wrench_ref = NaN; T_isa_wrench_ref = NaN;
    end
    
    for trial=1:nb_trials
        
        disp(['analyzing trial ' num2str(trial) '/' num2str(nb_trials) ' ...']);
        
        % Call class with measurements
        h = mean(diff(progress(:,trial))); % stepsize
        
        optim_class_result = object.calculate_invariants(wrench(:,:,trial),h,params);
        
        % Store results
        invars_wrench(:,:,trial) = optim_class_result.invariants;
        recons_wrench(:,:,trial) = [optim_class_result.Obj_rotation optim_class_result.Obj_translation];
        T_isa_wrench(:,:,:,trial) = optim_class_result.ISA_frames;
        
        result(counter).invars_wrench = invars_wrench(:,:,trial);
        result(counter).recons_wrench = recons_wrench(:,:,trial);
        result(counter).ISA_frames_wrench = T_isa_wrench(:,:,:,trial);
        result(counter).method = ocp_method;
        result(counter).trajectory_type = trajectory_type;
        result(counter).viewpoint = viewpoint; 
        result(counter).referencepoint = referencepoint; % {tracker, tool_point, force_sensor, middle_contour}
        result(counter).trial = trial;
        result(counter).measured_wrench = wrench(:,:,trial);
        result(counter).h = h;
        counter = counter + 1;
        
    end
    
    % Plotting results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    plot_all_results_screw('wrench',bool_reference_invariants,bool_visualize_trials,bool_visualize_reconstruction_errors,bool_visualize_summary,bool_paper_plots,...
        progress_ref,invars_wrench_ref,T_isa_wrench_ref,wrench_ref,viewpoint,referencepoint,parameterization,application,...
        recons_wrench_ref,nb_trials,progress,invars_wrench,T_isa_wrench,wrench,trial_0,recons_wrench,pose,pose_ref,trajectory_type,wrenchtype,path_to_data_folder)
    
    %% Generate movie
    for trial=1:nb_trials
        
        disp(['Generating movie ' num2str(trial) '/' num2str(nb_trials) ' ...']);
        fig = figure()
        axis_font_size = 12;
        label_font_size = 18;
        set(groot,'defaultAxesTickLabelInterpreter','latex');

        set(gcf, 'Position', get(0, 'Screensize'));
        T_isa_trial = T_isa_wrench(:,:,:,trial);
        N = size(T_isa_trial,3);
        step_size = h;
        
        for k = 1:N
       
            clf
        
            subplot(1,20,[1 2 3 4 5 6 7 8]);
            view_fig = [105,30];
            plot_ISA_frame_movie1(T_isa_wrench(:,:,:,trial),pose(:,:,:,trial),trial+trial_0-1,viewpoint,referencepoint,'wrench (Fig. 9b)',parameterization,application,view_fig,axis_font_size,label_font_size,step_size,wrenchtype,k)
            xlim([-0.4 0.2])
            ylim([1.4 1.6])
            zlim([-1.05 -0.8])

            
            % subplot(1,20,[12 13 14 15 16 17 18 19 20]);
            % view_fig = [105,30];
            % plot_ISA_frame_movie2(T_isa_wrench(:,:,:,trial),pose(:,:,:,trial),trial+trial_0-1,viewpoint,referencepoint,'wrench (Fig. 9b)',parameterization,application,view_fig,axis_font_size,label_font_size,step_size,wrenchtype,k)
            % xlim([-0.4 0.2])
            % ylim([1.4 1.6])
            % zlim([-1.05 -0.8])

            MovieFrames(k) = getframe(fig, [2 2 1535 1000]);  
            pause(0.1)
            100;
            
        end
        
        name = '../Output/video_mf_wrench_2';

        Writer = VideoWriter(name);
        Writer.FrameRate = 10;

        % Open the VideoWriter object, write the movie and close the file
        open(Writer);
        writeVideo(Writer,MovieFrames);
        close(Writer);
    end
    
end


%% Save the results in output folder
if ~isfolder('output')
    mkdir('output')
end
filename  = strcat(['output/result_' application '_screw_invariants_' trajectory_type '_seen_from_' viewpoint '_at_' referencepoint '.mat']);
save(filename,'result')

