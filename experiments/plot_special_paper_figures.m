function plot_special_paper_figures(results,settings_analysis)

% Figure 10a
if strcmp(settings_analysis.application,'contour') && strcmp(settings_analysis.trajectory_type,'pose') && strcmp(settings_analysis.ref_point_motion,'tracker') && settings_analysis.trial_n >= 5
    trial_nb = 5;
    trial = trial_nb-settings_analysis.trial_0+1;
    T_isa = results.trials(trial).moving_frames;
    pose_tcp = results.trials(trial).pose;
    plot_figure10a(T_isa,pose_tcp,trial)%,'motion (Fig. 10a)');
    %exportgraphics(gcf,['figures/ISA_frames_',datatype,'_',viewpoint,'_',referencepoint,'_trial_',num2str(trial+trial_0-1),'.pdf'],'ContentType','vector');
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