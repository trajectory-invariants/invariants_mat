function [temporal_wrench_trajectory] = apply_time_profile_to_geom_wrench_trajectory(geom_wrench_trajectory,time_xi)
% This function applies a time profile to the input geometric wrench
% trajectory.
%
% Input:
%     geom_wrench_trajectory = a wrench trajectory (Nx6) as a function of a
%                             geometric progress parameter xi 
%     time_xi      = The desired time profile [Nx2]. 
%                    The first column contains the elapsed time
%                    The second column contains the travelled progress.
%                    The matrix time_xi represents the time as a function of the travelled progress xi.
%
% Output:
%     temporal_wrench_trajectory = the temporal wrench trajectory (Nx6) 

N = size(geom_wrench_trajectory,1);

% Calculate time(xi) as a function of an equidistant xi (linear interpolation)
time_wrt_geom_progress = interp1(time_xi(:,2),time_xi(:,1),linspace(time_xi(1,2),time_xi(end,2),N));

% Find the temporal pose trajectory by linear interpolation
equi_time = linspace(time_xi(1,1),time_xi(end,1),N); % equidistant time vector
temporal_wrench_trajectory = interp1(time_wrt_geom_progress,geom_wrench_trajectory,equi_time);

end