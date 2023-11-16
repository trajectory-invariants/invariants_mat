function plot_all_results(results, settings_analysis, settings_plots)

% Settings
bool_plot_reference = settings_plots.plot_reference_results;
datatype = settings_analysis.trajectory_type;
parameterization = settings_analysis.progress_choice;

% Gather results from reference
invars_data_ref = results.reference.invariants;
progress_ref = results.reference.progress;
T_isa_data_ref = results.reference.moving_frames;
pose_ref = results.reference.pose;
trial_0 = settings_analysis.trial_0;

% Gather specific results from all trials in a multi-dimensional matrix
nb_trials = settings_analysis.trial_n - settings_analysis.trial_0 + 1; % number of trials
progress_all = reshape([results.trials(:).progress],[],nb_trials);
invariants_all = reshape([results.trials(:).invariants],[],6,nb_trials);

%% Plot invariants - individual
if settings_plots.plot_all_trials_invariants
    figure('Name',['screw invariants of ', datatype],'Color',[1 1 1],'NumberTitle','off');
    tabgroup_data_inv = uitabgroup;

    if bool_plot_reference
        thistab = uitab(tabgroup_data_inv,'Title','reference');
        plot_screw_invariants(progress_ref,invars_data_ref,datatype,parameterization,thistab);
    end

    for trial=1:nb_trials
        progress= results.trials(trial).progress;
        invars_data = results.trials(trial).invariants;
        thistab = uitab(tabgroup_data_inv,'Title',['trial = ',num2str(trial+trial_0-1)]);
        plot_screw_invariants(progress,invars_data,datatype,parameterization,thistab);
    end
end

%% Plot moving frames
if settings_plots.plot_all_trials_movingframes
    figure('Name',['ISA frames of ',datatype],'Color',[1 1 1],'NumberTitle','off');
    tabgroup_data_ISA = uitabgroup;

    for trial=1:nb_trials
        T_isa = results.trials(trial).moving_frames;
        pose_tcp = results.trials(trial).pose;
        thistab = uitab(tabgroup_data_ISA,'Title',['trial = ',num2str(trial+trial_0-1)]);
        plot_ISA_frames_tab(T_isa,pose_tcp,trial+trial_0-1,thistab)
    end

    if bool_plot_reference
        thistab = uitab(tabgroup_data_ISA,'Title','reference');
        plot_ISA_frames_tab(T_isa_data_ref,pose_ref,trial+trial_0-1,thistab)
    end


end

%% Plot invariants - summary
if settings_plots.plot_summary_invariants
    plot_screw_invariants_summary(bool_plot_reference,progress_ref,invars_data_ref,progress_all,invariants_all,datatype,parameterization)
end

%% Plot trajectory error
if plot_all_trials_trajectory_error
    figure('Name',['measured vs. reconstructed ',datatype],'Color',[1 1 1],'NumberTitle','off');
    tabgroup_data_recons = uitabgroup;
    if bool_plot_reference
        thistab = uitab(tabgroup_data_recons,'Title','reference'); axes(thistab);
        if strcmp(datatype,'pose')
            plot_trajectory_coordinates(progress_ref,[rotm2eul(data_ref(1:3,1:3,:),'zyx'),squeeze(data_ref(1:3,4,:))'],datatype,'measured vs. reconstructed pose',parameterization)
            plot_trajectory_coordinates(progress_ref,[rotm2eul(recons_data_ref(1:3,1:3,:),'zyx'),squeeze(recons_data_ref(1:3,4,:))'],datatype,'measured vs. reconstructed pose',parameterization)
        else
            plot_trajectory_coordinates(progress_ref,data_ref,datatype,'measured vs. reconstructed wrench',parameterization)
            plot_trajectory_coordinates(progress_ref,recons_data_ref,datatype,'measured vs. reconstructed wrench',parameterization)
        end
    end
    for trial=1:nb_trials
        thistab = uitab(tabgroup_data_recons,'Title',['trial = ',num2str(trial+trial_0-1)]); axes(thistab);
        if strcmp(datatype,'pose')
            plot_trajectory_coordinates(progress(:,trial),[rotm2eul(data(1:3,1:3,:,trial),'zyx'),squeeze(data(1:3,4,:,trial))'],datatype,'measured vs. reconstructed pose',parameterization)
            plot_trajectory_coordinates(progress(:,trial),[rotm2eul(recons_data(1:3,1:3,:,trial),'zyx'),squeeze(recons_data(1:3,4,:,trial))'],datatype,'measured vs. reconstructed pose',parameterization)
        else
            plot_trajectory_coordinates(progress(:,trial),data(:,:,trial),datatype,'measured vs. reconstructed wrench',parameterization)
            plot_trajectory_coordinates(progress(:,trial),recons_data(:,:,trial),datatype,'measured vs. reconstructed wrench',parameterization)
        end
    end
end

%% Special figures for the paper
if bool_paper_plots

    % Figure 10a
    if strcmp(application,'contour') && strcmp(trajectory_type,'motion') && strcmp(viewpoint,'world') && strcmp(referencepoint,'tracker') && trial_0 == 1 && nb_trials == 12
        trial = 5;
        view_fig = [175,16];
        axis_font_size = 24;
        label_font_size = 29;
        step_size = 2;
        plot_figure10(T_isa_data(:,:,:,trial),pose(:,:,:,trial),trial+trial_0-1,viewpoint,referencepoint,'motion (Fig. 9a)',parameterization,application,view_fig,axis_font_size,label_font_size,step_size,wrenchtype)
        exportgraphics(gcf,['figures/ISA_frames_',datatype,'_',viewpoint,'_',referencepoint,'_trial_',num2str(trial+trial_0-1),'.pdf'],'ContentType','vector');
    end

    % Figure 10b
    if strcmp(application,'contour') && strcmp(trajectory_type,'wrench') && strcmp(viewpoint,'body') && strcmp(referencepoint,'tracker') && trial_0 == 1 && nb_trials == 12
        trial = 5;
        view_fig = [-110,10];
        axis_font_size = 17;
        label_font_size = 22;
        step_size = 10;
        plot_figure10(T_isa_data(:,:,:,trial),pose(:,:,:,trial),trial+trial_0-1,viewpoint,referencepoint,'wrench (Fig. 9c)',parameterization,application,view_fig,axis_font_size,label_font_size,step_size,wrenchtype)
        exportgraphics(gcf,['figures/ISA_frames_',datatype,'_',viewpoint,'_',referencepoint,'_trial_',num2str(trial+trial_0-1),'.pdf'],'ContentType','vector');
    end

    % Figure 10d
    if strcmp(application,'contour') && strcmp(trajectory_type,'wrench') && strcmp(viewpoint,'world') && strcmp(referencepoint,'force_sensor') && trial_0 == 1 && nb_trials == 12
        trial = 5;
        view_fig = [170,22];
        axis_font_size = 34;
        label_font_size = 40;
        step_size = 3;
        plot_figure10(T_isa_data(:,:,:,trial),pose(:,:,:,trial),trial+trial_0-1,viewpoint,referencepoint,'wrench (Fig. 9b)',parameterization,application,view_fig,axis_font_size,label_font_size,step_size,wrenchtype)
        axis equal;
        exportgraphics(gcf,['figures/ISA_frames_',datatype,'_',viewpoint,'_',referencepoint,'_trial_',num2str(trial+trial_0-1),'.pdf'],'ContentType','vector');
    end

    % Figure 14a
    if strcmp(application,'peg') && strcmp(trajectory_type,'motion') && strcmp(viewpoint,'world') && strcmp(referencepoint,'tracker') && trial_0 == 1 && nb_trials == 12
        trial = 1;
        view_angles = [-100,20];
        plot_figure14(T_isa_data(:,:,:,trial), pose(:,:,:,trial), trial, referencepoint,view_angles)
        exportgraphics(gcf,['figures/ISA_frames_',datatype,'_',viewpoint,'_',referencepoint,'_trial_',num2str(trial),'_peg.pdf'],'ContentType','vector');
    end

    % Figure 14b
    if strcmp(application,'peg') && strcmp(trajectory_type,'wrench') && strcmp(viewpoint,'world') && strcmp(referencepoint,'force_sensor') && trial_0 == 1 && nb_trials == 12
        trial = 1;
        view_angles = [-100,20];
        plot_figure14(T_isa_data(:,:,:,trial), pose(:,:,:,trial), trial, referencepoint,view_angles)
        exportgraphics(gcf,['figures/ISA_frames_',datatype,'_',viewpoint,'_',referencepoint,'_trial_',num2str(trial),'_peg.pdf'],'ContentType','vector');
        trial = 12;
        view_angles = [166,20];
        plot_figure14(T_isa_data(:,:,:,trial), pose(:,:,:,trial), trial, referencepoint,view_angles)
        exportgraphics(gcf,['figures/ISA_frames_',datatype,'_',viewpoint,'_',referencepoint,'_trial_',num2str(trial),'_peg.pdf'],'ContentType','vector');
    end
end

