function [reconstructed_wrench,reconstructed_ISA_frame] = reconstruct_wrench_trajectory_from_screw_invariants(screw_invariants,initial_ISA_frame_wrench,dt,N)
% Reconstruct the object pose trajectory from the supplied screw axis invariants
%
% Input:  screw_invariants       = screw axis invariants (Nx6) [f|omega_kappa|omega_tau|m|v_b|v_t]
%         initial_ISA_frame_pose = initial pose matrix of the ISA frame (4x4)
%         dt                     = integration step
% Output: reconstructed_wrench   = reconstructed contact wrench (Nx6)
%         reconstructed_ISA_frame= reconstructed ISA frames (4x4xN)

    % initialize the output
    reconstructed_ISA_frame = zeros(4,4,N);
    reconstructed_ISA_frame(:,:,1) = initial_ISA_frame_wrench;
    reconstructed_wrench = zeros(6,N);

    % reconstruct the wrench trajectory by open-loop integration
    for k = 1:N-1
        [T_isa_plus1,invariants_obj_world] = integrator_screw_invariants_to_screw(...
            reconstructed_ISA_frame(1:3,:,k),screw_invariants(k,:),dt);
        reconstructed_ISA_frame(1:3,:,k+1) = T_isa_plus1;
        reconstructed_ISA_frame(4,4,k+1) = 1;
        reconstructed_wrench(:,k) = invariants_obj_world;
    end
    % last sample
    reconstructed_wrench(:,end) = transform_screw(T_isa_plus1,[screw_invariants(N,1);0;0;screw_invariants(N,4);0;0]);
    reconstructed_wrench = reconstructed_wrench';
end