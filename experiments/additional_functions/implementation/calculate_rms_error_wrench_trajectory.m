function [rms_diff_force,rms_diff_moment] = calculate_rms_error_wrench_trajectory(wrench_ref,wrench_input)
% Calculate the rms difference between the force trajectories and
% moment trajectories of the wrench trajectories: wrench_ref and wrench_input
%
% Input:  wrench_ref      = reference wrench trajectory (e.g. measurements) (Nx6)
%         wrench_input    = input wrench trajectory (Nx6). 
%                           This trajectory is compared to the reference trajectory 
% Output: rms_error_force    = calculated rms error on the force trajectory
%         rms_error_wrench   = calculated rms error on the wrench trajectory

N = size(wrench_ref,1);

% calculate the rms error of the reconstructed force and moment trajectories
rms_diff_force = sqrt(sum((wrench_input(1:end,1:3)-wrench_ref(1:end,1:3)).^2,'all')/N);
% calculate the rms error of the reconstructed moment trajectory
rms_diff_moment = sqrt(sum((wrench_input(1:end,4:6)-wrench_ref(1:end,4:6)).^2,'all')/N);

end