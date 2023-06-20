close all; 
clear; clc; restoredefaultpath
addpath(genpath('../implementation/'));
path_to_data_folder = '../data/';

%% This script calculates screw invariants from average wrench trajectories
% This averaging of the wreng trajectories results in a reduced sensitivity to measurement noise 
% The results are stored in the output folder

% Settings 
application = 'peg'; % {contour, peg}
trajectory_type = 'wrench'; % {motion, wrench}
wrenchtype = 'real'; % {real, synthetic}

%% Parameters
% Parameters for tuning
if strcmp(application,'peg')
    rms_error_pos = 0.002; % [mm]
    rms_error_rot = 2*pi/180; % 2 [degrees] converted to [rad]
    rms_error_force = 0.17; % [N]
    rms_error_moment = 0.05; % [Nm]
    L = 0.5; % [m] global scale to weigh the rotational and translational moving frame invariants
else
    error('Wrong application selected')
end

% Parameters for invariants sign
params.positive_obj_invariant = 0;
params.positive_mov_invariant = 0;

% Parameterization of the analysis
parameterization = 'dimless_arclength'; % {time_based, dimless_arclength}

% Parameters of input data
N = 101;
batch_0 = linspace(1,12,12);
batch_1 = linspace(1,7,7);
batch_2 = linspace(8,12,5);

if ~isfolder('../output')
    mkdir('output')
end

% Parameters optimization problem
params.weights.rms_error_orientation = rms_error_force;
params.weights.rms_error_translation = rms_error_moment;
params.weights.L = L; % scaling for making positions dimensionless
params.window.window_length = N;

progress = linspace(0,1,N);

% Call class with measurements
h = mean(diff(progress)); % stepsize
        
% Build the optimization problem
object = OCP_calculate_screw_invariants_wrench(params);

% Solve the optimization problem for different batches and configurations
for batch_number = 0:2
    for viewpoints = {'world','body'}
        
        viewpoint = viewpoints{1};
        if strcmp(viewpoint,'world')
            referencepoint = 'force_sensor';
        else
            referencepoint = 'tracker';
        end
        
        if batch_number == 0
            batch = batch_0;
        elseif batch_number == 1
            batch = batch_1;
        else
            batch = batch_2;
        end
        
        check_input_screw(trajectory_type,viewpoint,referencepoint);

        % Retrieve the wrench data

        [bool_motion,bool_force] = check_screw_analysis_type(trajectory_type);

        % Load data
        [~,pose,position,rotation,wrench,force,moment,...
            progress_ref,pose_ref,position_ref,rotation_ref,wrench_ref,force_ref,moment_ref] = ...
            contour_preprocess_data(N,viewpoint,parameterization,referencepoint,1,12,application,wrenchtype,path_to_data_folder);

        if strcmp(viewpoint,'world')
            wrench_ref = -wrench_ref; force_ref = -force_ref; moment_ref = -moment_ref;
            wrench = -wrench; force = -force; moment = -moment;
        end

        % average the wrench trajectories over the desired batch
        counter = 1;
        mean_wrench = zeros(N,6);
        nb_trials = size(batch,2);
        if bool_force
            for trial=batch
                mean_wrench = mean_wrench + wrench(:,:,trial);
            end
        end
        mean_wrench = mean_wrench/nb_trials;

        %% Calculate the invariant descriptors
        optim_class_result = object.calculate_invariants(mean_wrench,h);

        % Store results
        invars_wrench = optim_class_result.invariants;
        recons_wrench = [optim_class_result.Obj_rotation optim_class_result.Obj_translation];
        T_isa_wrench = optim_class_result.ISA_frames;

        result.invars_wrench = invars_wrench;
        result.recons_wrench = recons_wrench;
        result.T_isa_wrench = T_isa_wrench;
        result.viewpoint = viewpoint;
        result.trajectory_type = trajectory_type;

        %% Save the results in output folder
        filename  = strcat(['../output/result_' application '_screw_invariants_' trajectory_type '_seen_from_' viewpoint '_mean_over_batch_' num2str(batch_number) '.mat']);
        save(filename,'result')
    end
end

%% In this next sep, the screw invariants calculated from the average wrench trajectories are retrieved and plotted 

application = 'peg';
trajectory_type = 'wrench'; % {motion,wrench} for screw invariants
N = 101;
progress = linspace(0,1,N);

% Initialize figure and its position/size
figure1 = figure('Color',[1 1 1],'NumberTitle','off');
set(gcf,'Units','normalized','OuterPosition',[0.2448    0.4352    0.7568    0.3657]);
set(groot,'defaultAxesTickLabelInterpreter','latex');

for batch_number = 0:2
    if batch_number == 0
        color = 'g';
    elseif batch_number == 1
        color = 'r';
    elseif batch_number == 2
        color = 'b';
    end
    for viewpoints = 1:2
        if viewpoints == 1
            viewpoint = 'world';
            line_style = '-';
        else
            viewpoint = 'body';
            line_style = '--';
        end
        
        filename  = strcat(['../output/result_' application '_screw_invariants_' trajectory_type '_seen_from_' viewpoint '_mean_over_batch_' num2str(batch_number) '.mat']);
        result = load(filename);
        
        line_spec = strcat([color line_style]);
        y_limits = [-1,4;-0.1 1; -1, 1; -0.05 0.05; -0.01 0.01; -0.01 0.01];
        titles = {'$a$ [N]','$\omega_\kappa$ [rad/-]','$\omega_\tau$ [rad/-]','$b$ [Nm]','$v_b$ [m/-]','$v_t$ [m/-]'};
        for k = 1:2
            subplots(k) = subplot(1,3,k,'YGrid','on','FontSize',22);
            box(subplots,'on');
            hold(subplots,'all');
            plot(progress,result.result.invars_wrench(:,k),line_spec,'LineWidth',2)
            %set(gca,'FontSize',15)
            xlim([0 1])
            ylim(y_limits(k,:))
            xlabel('$\xi$ [-]','Interpreter','LaTex','FontSize',22,'Rotation',0,'HorizontalAlignment', 'left');
            title(titles{k},'interpreter','latex','FontSize',26)
            hold on
        end
        subplot(1,3,3)
        plot(0,0,line_spec,'LineWidth',2)
        grid off
        axis off
        hold on
    end
end

legend(' mean of all trials, seen from world',' mean of all trials, seen from tool',...
            ' mean of batch 1, seen from world',' mean of batch 1, seen from tool',...
            ' mean of batch 2, seen from world',' mean of batch 2, seen from tool','FontSize',20,'location','eastoutside','Interpreter','latex')
legend boxoff 

exportgraphics(figure1,['../figures/peg_on_hole_analysis.pdf'],'ContentType','vector');



