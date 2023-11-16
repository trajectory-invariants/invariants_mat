function [bool_motion,bool_force] = check_screw_analysis_type(trajectory_type)

if strcmp(trajectory_type,'motion')
    bool_motion = 1; % {0,1}
    bool_force = 0; % {0,1}
end
if strcmp(trajectory_type,'wrench')
    bool_motion = 0; % {0,1}
    bool_force = 1; % {0,1}
end
