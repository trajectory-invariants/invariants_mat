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

%% Plot trajectory error
if settings_plots.plot_all_trials_trajectory_errors
    figure('Name',['measured vs. reconstructed ',datatype],'Color',[1 1 1],'NumberTitle','off');
    tabgroup_data_recons = uitabgroup;

    if bool_plot_reference
        thistab = uitab(tabgroup_data_recons,'Title','reference'); axes(thistab);
        data_ref = results.reference.measured_trajectory;
        plot_trajectory_coordinates(progress_ref,data_ref,datatype,parameterization)
        recons_data_ref = results.reference.reconstructed_trajectory;
        plot_trajectory_coordinates(progress_ref,recons_data_ref,datatype,parameterization)
    end

    for trial=1:nb_trials
        thistab = uitab(tabgroup_data_recons,'Title',['trial = ',num2str(trial+trial_0-1)]); axes(thistab);
        progress = results.trials(trial).progress;
        data = results.trials(trial).measured_trajectory;
        recons_data = results.trials(trial).reconstructed_trajectory;
        plot_trajectory_coordinates(progress,data,datatype,parameterization)
        plot_trajectory_coordinates(progress,recons_data,datatype,parameterization)
    end
end

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
        pose_tcp = results.trials(trial).pose_tcp;
        thistab = uitab(tabgroup_data_ISA,'Title',['trial = ',num2str(trial+trial_0-1)]);
        plot_ISA_frames_tab(T_isa,pose_tcp,trial+trial_0-1,thistab)
    end


    if bool_plot_reference
        thistab = uitab(tabgroup_data_ISA,'Title','reference');
        plot_ISA_frames_tab(T_isa_data_ref,pose_ref,0,thistab)
    end


end

%% Plot invariants - summary
if settings_plots.plot_summary_invariants
    plot_screw_invariants_summary(bool_plot_reference,progress_ref,invars_data_ref,progress_all,invariants_all,datatype,parameterization)

    if strcmp(datatype,'wrench') && strcmp(viewpoint,'body') && strcmp(wrenchtype,'real')
        subplot(2,3,5); ylim([-0.2,0.2]); subplot(2,3,6); ylim([-0.1,0.1]);
        exportgraphics(gcf,['figures/screw_invariants_',datatype,'_',viewpoint,'_',referencepoint,'.pdf'],'ContentType','vector');
    end
    if strcmp(datatype,'wrench') && strcmp(wrenchtype,'synthetic')
        subplot(2,3,1); ylim([-1,35]); subplot(2,3,2); ylim([-5,5]); %subplot(2,3,3); ylim([-5,5])
        exportgraphics(gcf,['figures/screw_invariants_',datatype,'_',viewpoint,'_',referencepoint,'_',wrenchtype,'.pdf'],'ContentType','vector');
    end
end
