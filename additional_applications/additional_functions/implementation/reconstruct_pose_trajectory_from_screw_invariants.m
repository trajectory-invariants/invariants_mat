function [reconstructed_Obj_pose,reconstructed_ISA_frame] = reconstruct_pose_trajectory_from_screw_invariants(screw_invariants,initial_Obj_pose,initial_ISA_frame_pose,dt,N)
% Reconstruct the object pose trajectory from the supplied screw axis invariants
%
% Input:  screw_invariants       = screw axis invariants (Nx6) [omega|omega_kappa|omega_tau|v|v_b|v_t]
%         initial_Obj_pose       = initial pose matrix of the object (4x4)
%         initial_ISA_frame_pose = initial pose matrix of the ISA frame (4x4)
%         dt                     = integration step
% Output: reconstructed_Obj_pose = reconstructed object frame (4x4xN)
%         reconstructed_ISA_frame= reconstructed ISA frames (4x4xN)

    % initialize output
    reconstructed_ISA_frame = zeros(4,4,N);
    reconstructed_Obj_pose = zeros(4,4,N);
    reconstructed_ISA_frame(:,:,1) = initial_ISA_frame_pose;
    reconstructed_Obj_pose(:,:,1) = initial_Obj_pose;

    % reconstruct the rigid body trajectory by open-loop integration
    for k = 1:N-1
        [T_isa_plus1,T_obj_plus1] = integrator_screw_invariants_to_pose(...
            reconstructed_ISA_frame(1:3,:,k),reconstructed_Obj_pose(1:3,:,k),...
            screw_invariants(k,:),dt);
        reconstructed_Obj_pose(1:3,:,k+1) = T_obj_plus1;
        reconstructed_Obj_pose(4,4,k+1) = 1;
        reconstructed_ISA_frame(1:3,:,k+1) = T_isa_plus1;
        reconstructed_ISA_frame(4,4,k+1) = 1;
    end
end