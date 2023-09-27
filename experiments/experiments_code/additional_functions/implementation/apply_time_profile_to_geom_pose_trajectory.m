function [temporal_pose_trajectory] = apply_time_profile_to_geom_pose_trajectory(geom_pose_trajectory,time_xi)
% This function applies a time profile to the input geometric pose
% trajectory.
%
% Input:
%     geom_pose_trajectory = a pose trajectory (4x4xN) as a function of a
%                             geometric progress parameter xi 
%     time_xi      = The desired time profile [Nx2]. 
%                    The first column contains the elapsed time
%                    The second column contains the travelled progress.
%                    The matrix time_xi represents the time as a function of the travelled progress xi.
%
% Output:
%     temporal_pose_trajectory = the temporal pose trajectory (4x4xN) 

N = size(geom_pose_trajectory,3);

% Calculate time(xi) as a function of an equidistant xi (linear interpolation)
time_wrt_geom_progress = interp1(time_xi(:,2),time_xi(:,1),linspace(time_xi(1,2),time_xi(end,2),N));

% Find the temporal pose trajectory by linear interpolation
equi_time = linspace(time_xi(1,1),time_xi(end,1),N); % equidistant time vector
temporal_pose_trajectory = interp_pose(time_wrt_geom_progress,geom_pose_trajectory,equi_time);

end