function [rms_diff_pos,rms_diff_orientation] = calculate_rms_error_pose_trajectory(T_ref,T_input)
% Calculate the rms difference between the position trajectories and
% orientation trajectories of the pose trajectories T_ref and T_input
%
% Input:  T_ref      = reference pose trajectory (e.g. measurements) (4x4xN)
%         T_input    = input pose trajectory (4x4xN). 
%                      This trajectory is compared to the reference trajectory 
% Output: rms_error_pos           = calculated rms error on the position trajectory
%         rms_error_orientation   = calculated rms error on the orientation trajectory

N = size(T_ref,3);

% calculate the rms error of the reconstructed position trajectory
pos_ref = squeeze(T_ref(1:3,4,:))';
pos_input = squeeze(T_input(1:3,4,:))';
rms_diff_pos = sqrt(sum((pos_input-pos_ref).^2,'all')/N);

% calculate rms error of the reconstructed orientation trajectory
R_ref = T_ref(1:3,1:3,:);
R_input = T_input(1:3,1:3,:);
rms_diff_orientation = 0;
for k = 1:N
    diff_matrix = R_ref(:,:,k)'*R_input(:,:,k)-eye(3);
    rms_diff_orientation = rms_diff_orientation + diff_matrix(1,1)^2+diff_matrix(1,2)^2+diff_matrix(1,3)^2 ...
        + diff_matrix(2,2)^2+diff_matrix(2,3)^2+diff_matrix(3,3)^2;        
end
rms_diff_orientation = sqrt(rms_diff_orientation/N);
end