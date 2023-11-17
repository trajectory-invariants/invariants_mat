function [] = plot_reconstruction_results(pose_time,wrench_time,T_global,transformed_pose_time,...
    reconstructed_Obj_pose_time,reconstructed_ISA_frame_pose,reconstructed_wrench_time,reconstructed_ISA_frame_wrench,trial)

    % transform wrench measurements to new location (this is only necessary for the plots!)
    N = size(pose_time,3);
    transformed_wrench_time = zeros(6,N);
    for k = 1:N
        transformed_wrench_time(:,k) = transform_screw(T_global,wrench_time(k,:)');
    end

    %% Transform the camera angle (for visualization)
    R_visualization = rot_x(90);
    T_visualization = [R_visualization,[0;0;0];0 0 0 1];
    P_visualization = [R_visualization,zeros(3,3);zeros(3,3),R_visualization];
    for k = 1:N
        pose_time(:,:,k) = T_visualization*pose_time(:,:,k);
        transformed_pose_time(:,:,k) = T_visualization*transformed_pose_time(:,:,k);
        reconstructed_Obj_pose_time(:,:,k) = T_visualization*reconstructed_Obj_pose_time(:,:,k);
        
        wrench_time(k,:) = wrench_time(k,:)*P_visualization';
        reconstructed_wrench_time(k,:) = reconstructed_wrench_time(k,:)*P_visualization';
        transformed_wrench_time(:,k) = P_visualization*transformed_wrench_time(:,k);
        
        T_global = T_visualization*T_global;
        reconstructed_ISA_frame_pose(:,:,k) = T_visualization*reconstructed_ISA_frame_pose(:,:,k);
        reconstructed_ISA_frame_wrench(:,:,k) = T_visualization*reconstructed_ISA_frame_wrench(:,:,k);
    end

    % Figure 11a
    plot_reconstruction_screw_traj_contour(pose_time,wrench_time,T_global,...
        reconstructed_Obj_pose_time,reconstructed_ISA_frame_pose,...
        reconstructed_wrench_time',reconstructed_ISA_frame_wrench,trial)
    % exportgraphics(gcf,'../figures/reconstruction_screw_trajectories.pdf','ContentType','vector');

    % Figure 11b
    plot_transformed_screw_traj_contour(pose_time,wrench_time,...
        transformed_pose_time,transformed_wrench_time,trial)
    zlim([-0.5993   -0.2511]);

end