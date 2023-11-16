function [positions_geom,stepsize,s_n] = reparameterize_positiontrajectory(measured_trajectory,N_des)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

if nargin < 2
    N_des = size(measured_trajectory,1);
end

time = measured_trajectory(:,1);
positions = measured_trajectory(:,2:4);


pdiff = sqrt(sum(diff(positions).^2,2));
s = [ 0 ; cumsum(abs(pdiff))];
[~, idx_unique, ~] = unique(round(positions,10),'rows','stable');

s_n = linspace(0,s(idx_unique(end)),N_des); % linspace(0,s(end),N_des);
positions_geom = interp1(s(idx_unique),positions(idx_unique,:),s_n);

s_normalized = s./s(end);

stepsize = 1/length(s_n);


end