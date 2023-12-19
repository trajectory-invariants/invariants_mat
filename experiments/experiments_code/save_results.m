function save_results(results, settings_analysis, settings_plots)

%% Save the results in output folder
if ~isfolder('results')
    mkdir('results')
end
%filename  = strcat(['output/result_' application '_screw_invariants_' trajectory_type '_seen_from_' viewpoint '_at_' referencepoint '.mat']);
filename  = strcat(['results/result_' settings_analysis.trajectory_type '.mat']);

save(filename,'results')
