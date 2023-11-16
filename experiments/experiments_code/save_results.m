%% Save the results in output folder
if ~isfolder('output')
    mkdir('output')
end
filename  = strcat(['output/result_' application '_screw_invariants_' trajectory_type '_seen_from_' viewpoint '_at_' referencepoint '.mat']);
save(filename,'result')
