function movie_moving_frame(results, settings_analysis)

trial_0 = settings_analysis.trial_0;

% Gather specific results from all trials in a multi-dimensional matrix
nb_trials = settings_analysis.trial_n - settings_analysis.trial_0 + 1; % number of trials

%% Generate movie moving frame
fig = figure();
set(gcf, 'Position', get(0, 'Screensize'));
hold on;

for trial=1:nb_trials
    T_isa = results.trials(trial).moving_frames;
    pose_tcp = results.trials(trial).pose_tcp;
    
    N = size(T_isa,3);
    for k = 1:N
        
        clf
        
        subplot(1,10,[1 2 3 5]);
        plot_ISA_frame(T_isa(:,:,k),pose_tcp,trial+trial_0-1,k,[120,15])
        
        subplot(1,10,[7 8 9]);
        plot_ISA_frame(T_isa(:,:,k),pose_tcp,trial+trial_0-1,k,[90,90])
        
        MovieFrames(k) = getframe(fig, [2 2 1535 791]);  
        pause(0.1)
        
    end
    
    name = '../../Output/video_mf';

    Writer = VideoWriter(name);
    Writer.FrameRate = 10;

    % Open the VideoWriter object, write the movie and close the file
    open(Writer);
    writeVideo(Writer,MovieFrames);
    close(Writer);
    
end

end

