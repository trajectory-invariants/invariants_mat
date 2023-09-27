function [measured_pose,recons_pose,ISA_frames_pose,invars_pose,h,N] = retrieve_data_calculated_screw_invariants_motion(application,trajectory_type,viewpoint,referencepoint,trial)
    
    % retrieve the data
    filename  = strcat(['../output/result_' application '_screw_invariants_' trajectory_type '_seen_from_' viewpoint '_at_' referencepoint '.mat']);
    result = load(filename);

    % select a trial (In the paper, the rigid body trajectory of the tool of trial 7 was chosen)
    if strcmp(result.result(1).trial,'ref') 
        trial = trial + 1;
    end
    measured_pose = result.result(trial).measured_pose;

    N = size(measured_pose,3);
    
    % Initialize output
    recons_pose = zeros(4,4,N);
    ISA_frames_pose = zeros(4,4,N);
    for k = 1:N
        recons_pose(1:3,:,k) = result.result(trial).recons_pose(:,:,k);
        recons_pose(4,4,k) = 1;
        ISA_frames_pose(1:3,:,k) = result.result(trial).ISA_frames_pose(:,:,k);
        ISA_frames_pose(4,4,k) = 1;
    end
    invars_pose = result.result(trial).invars_pose;
    h = result.result(trial).h;
end