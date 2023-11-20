function [label_x, label_y] = axes_labels_trajectory(datatype, parameterization)

% Pick the correct labels for the X-axis based on the parameterization
if strcmp(parameterization,'arclength_normalized')
    label_x = '$\xi\ [\mathrm{-}]$';
elseif strcmp(parameterization,'time_based')
    label_x = '$t\ [\mathrm{s}]$';
elseif strcmp(parameterization,'arclength')
    label_x = '$s$ [m]';
else % generic labels
    label_x = '$\xi$';
end

% Pick the correct labels for the X-axis based on the data type
if strcmp(datatype,'pose')
    label_y = {'$roll\ [\mathrm{rad}]$','$pitch\ [\mathrm{rad}]$','$yaw\ [\mathrm{rad}]$','$p_x\ [\mathrm{m}]$','$p_y\ [\mathrm{m}]$','$p_z\ [\mathrm{m}]$'};
elseif strcmp(datatype,'wrench')
    label_y = {'$f_x\ [\mathrm{N}]$','$f_y\ [\mathrm{N}]$','$f_z\ [\mathrm{N}]$','$m_x\ [\mathrm{N.m}]$','$m_y\ [\mathrm{N.m}]$','$m_z\ [\mathrm{N.m}]$'};
elseif strcmp(datatype,'position')
    label_y = {'$p_x\ [\mathrm{m}]$','$p_y\ [\mathrm{m}]$','$p_z\ [\mathrm{m}]$'};
elseif strcmp(datatype,'orientation')
    label_y = {'$roll\ [\mathrm{rad}]$','$pitch\ [\mathrm{rad}]$','$yaw\ [\mathrm{rad}]$'};
elseif strcmp(datatype,'force')
    label_y = {'$f_x\ [\mathrm{N}]$','$f_y\ [\mathrm{N}]$','$f_z\ [\mathrm{N}]$'};
elseif strcmp(datatype,'moment')
    label_y = {'$m_x\ [\mathrm{N.m}]$','$m_y\ [\mathrm{N.m}]$','$m_z\ [\mathrm{N.m}]$'};
else % generic labels
    label_y = {'$i_1$','$i_2$','$i_3$','$i_4$','$i_5$','$i_6$'};
end