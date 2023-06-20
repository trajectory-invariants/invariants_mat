function plot_all_results_vector(datatype,bool_reference_invariants,bool_visualize_trials,bool_visualize_reconstruction_errors,bool_visualize_summary,bool_paper_plots,...
    progress_ref,invars_data_ref,R_FS_data_ref,pose_ref,viewpoint,referencepoint,parameterization,application,data_ref,...
    recons_data_ref,nb_trials,progress,invars_data,R_FS_data,trial_0,data,recons_data,pose,trajectory_type,wrenchtype,path_to_data_folder)

%%
if bool_reference_invariants
    % Plot vector invariants of reference data
    fig_data_inv = figure('Name',['vector invariants of ',datatype],'Color',[1 1 1],'NumberTitle','off');
    tabgroup_data_inv = uitabgroup; thistab = uitab(tabgroup_data_inv,'Title','reference');
    plot_vector_invariants(progress_ref,invars_data_ref,datatype,parameterization,thistab)
    % Plot FS frames of reference data
    fig_data_FS = figure('Name',['FS frames of ',datatype],'Color',[1 1 1],'NumberTitle','off');
    tabgroup_data_FS = uitabgroup; thistab = uitab(tabgroup_data_FS,'Title','reference');
    plot_FS_frames_contour_tab(R_FS_data_ref,invars_data_ref(:,1),pose_ref,'ref',viewpoint,referencepoint,datatype,parameterization,application,thistab,wrenchtype,path_to_data_folder);
end

% Plot reference and reconstructed data
if bool_reference_invariants && bool_visualize_reconstruction_errors
    fig_data_recons = figure('Name',['measured vs. reconstructed ',datatype],'Color',[1 1 1],'NumberTitle','off');
    tabgroup_data_recons = uitabgroup; thistab = uitab(tabgroup_data_recons,'Title','reference'); axes(thistab);
    if strcmp(datatype,'orientation')
        plot_trajectory_coordinates(progress_ref,rotm2eul(data_ref,'zyx'),datatype,['measured vs. reconstructed ',datatype],parameterization)
        plot_trajectory_coordinates(progress_ref,rotm2eul(recons_data_ref,'zyx'),datatype,['measured vs. reconstructed ',datatype],parameterization)
    else
        plot_trajectory_coordinates(progress_ref,data_ref,datatype,['measured vs. reconstructed ',datatype],parameterization)
        plot_trajectory_coordinates(progress_ref,recons_data_ref,datatype,['measured vs. reconstructed ',datatype],parameterization)
    end
end

% Plot vector invariants of demonstrated data
if bool_visualize_trials
    for trial=1:nb_trials
        if ~bool_reference_invariants
            if trial == 1
                fig_data_inv = figure('Name',['vector invariants of ',datatype],'Color',[1 1 1],'NumberTitle','off');
                tabgroup_data_inv = uitabgroup; thistab = uitab(tabgroup_data_inv,'Title',['trial = ',num2str(trial+trial_0-1)]);
            else
                figure(fig_data_inv); thistab = uitab(tabgroup_data_inv,'Title',['trial = ',num2str(trial+trial_0-1)]);
            end
        elseif bool_reference_invariants
            figure(fig_data_inv); thistab = uitab(tabgroup_data_inv,'Title',['trial = ',num2str(trial+trial_0-1)]);
        end
        plot_vector_invariants(progress(:,trial),invars_data(:,:,trial),datatype,parameterization,thistab)
    end
end

% Plot FS frames of demonstrated data
if bool_visualize_trials
    for trial=1:nb_trials
        if ~bool_reference_invariants
            if trial == 1
                fig_data_FS = figure('Name',['FS frames of ',datatype],'Color',[1 1 1],'NumberTitle','off');
                tabgroup_data_FS = uitabgroup; thistab = uitab(tabgroup_data_FS,'Title',['trial = ',num2str(trial+trial_0-1)]);
            else
                figure(fig_data_FS); thistab = uitab(tabgroup_data_FS,'Title',['trial = ',num2str(trial+trial_0-1)]);
            end
        elseif bool_reference_invariants
            figure(fig_data_FS); thistab = uitab(tabgroup_data_FS,'Title',['trial = ',num2str(trial+trial_0-1)]);
        end
        plot_FS_frames_contour_tab(R_FS_data(:,:,:,trial),invars_data(:,1,trial),pose(:,:,:,trial),trial+trial_0-1,viewpoint,referencepoint,datatype,parameterization,application,thistab,wrenchtype,path_to_data_folder);
    end
end

% Plot measured and reconstructed data
if bool_visualize_reconstruction_errors
    for trial=1:nb_trials
        if ~bool_reference_invariants
            if trial == 1
                fig_data_recons = figure('Name',['measured vs. reconstructed ',datatype],'Color',[1 1 1],'NumberTitle','off');
                tabgroup_data_recons = uitabgroup; thistab = uitab(tabgroup_data_recons,'Title',['trial = ',num2str(trial+trial_0-1)]); axes(thistab);
            else
                figure(fig_data_recons); thistab = uitab(tabgroup_data_recons,'Title',['trial = ',num2str(trial+trial_0-1)]); axes(thistab);
            end
        elseif bool_reference_invariants
            figure(fig_data_recons); thistab = uitab(tabgroup_data_recons,'Title',['trial = ',num2str(trial+trial_0-1)]); axes(thistab);
        end
        if strcmp(datatype,'orientation')
            plot_trajectory_coordinates(progress(:,trial),rotm2eul(data(:,:,:,trial),'zyx'),datatype,['measured vs. reconstructed ',datatype],parameterization)
            plot_trajectory_coordinates(progress(:,trial),rotm2eul(recons_data(:,:,:,trial),'zyx'),datatype,['measured vs. reconstructed ',datatype],parameterization)
        else
            plot_trajectory_coordinates(progress(:,trial),data(:,:,trial),datatype,['measured vs. reconstructed ',datatype],parameterization)
            plot_trajectory_coordinates(progress(:,trial),recons_data(:,:,trial),datatype,['measured vs. reconstructed ',datatype],parameterization)
        end
    end
end

% Plot reference and demonstrated invairnats
if bool_visualize_summary
    plot_vector_invariants_contour(progress_ref,invars_data_ref,progress,invars_data,datatype,parameterization);
    exportgraphics(gcf,['figures/vector_invariants_',datatype,'_',viewpoint,'_',referencepoint,'.pdf'],'ContentType','vector');
end

%% Special figures for the paper
if bool_paper_plots
    % Figure 10d
    if strcmp(application,'contour') && strcmp(trajectory_type,'position') && strcmp(viewpoint,'world') && strcmp(referencepoint,'tool_point') && trial_0 == 1 && nb_trials == 12
        trial = 5;
        view_fig = [170,22];
        axis_font_size = 34;
        label_font_size = 40;
        step_size = 9;
        plot_FS_frames_contour(R_FS_data(:,:,:,trial),invars_data(:,1,trial),pose(:,:,:,trial),trial+trial_0-1,viewpoint,referencepoint,'position (Fig. 9d)',parameterization,application,view_fig,axis_font_size,label_font_size,step_size,wrenchtype);
        axis equal;
        xlim([-0.3748,0.1797]);ylim([1.4064,1.6116]);zlim([-1.058,-0.7751]);
        exportgraphics(gcf,['figures/FS_frames_',datatype,'_',viewpoint,'_',referencepoint,'_trial_',num2str(trial+trial_0-1),'.pdf'],'ContentType','vector');
% xlim
% 
% ans =
% 
%    -0.3748    0.1797
% 
% ylim
% 
% ans =
% 
%     1.4064    1.6116
% 
% zlim
% 
% ans =
% 
%    -1.0581   -0.7751

    
    end
end
