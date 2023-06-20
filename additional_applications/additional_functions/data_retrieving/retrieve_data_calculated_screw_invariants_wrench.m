function [measured_wrench,recons_wrench,ISA_frames_wrench,invars_wrench,h,N] = retrieve_data_calculated_screw_invariants_wrench(application,trajectory_type,viewpoint,referencepoint,trial)
    
    % retrieve the screw invariants
    filename  = strcat(['../output/result_' application '_screw_invariants_' trajectory_type '_seen_from_' viewpoint '_at_' referencepoint '.mat']);
    result = load(filename);

    % select a trial (In the paper, the wrench trajectory of the tool of trial 7 was chosen)
    if strcmp(result.result(1).trial,'ref')
        trial = trial + 1;
    end

    measured_wrench = result.result(trial).measured_wrench;  
    N = size(measured_wrench,1);
    recons_wrench = result.result(trial).recons_wrench;
    invars_wrench = result.result(trial).invars_wrench;
    
    % initialize output
    ISA_frames_wrench = zeros(4,4,N);
    for k = 1:N
        ISA_frames_wrench(1:3,:,k) = result.result(trial).ISA_frames_wrench(:,:,k);
        ISA_frames_wrench(4,4,k) = 1;
    end
    h = result.result(trial).h;
end