function [bool_position,bool_orientation,bool_force,bool_moment] = check_vector_analysis_type(trajectory_type)

if strcmp(trajectory_type,'position')
    bool_position = 1; % {0,1}
    bool_orientation = 0; % {0,1}
    bool_force = 0; % {0,1}
    bool_moment = 0; % {0,1}
end
if strcmp(trajectory_type,'orientation')
    bool_position = 0; % {0,1}
    bool_orientation = 1; % {0,1}
    bool_force = 0; % {0,1}
    bool_moment = 0; % {0,1}
end
if strcmp(trajectory_type,'force')
    bool_position = 0; % {0,1}
    bool_orientation = 0; % {0,1}
    bool_force = 1; % {0,1}
    bool_moment = 0; % {0,1}
end
if strcmp(trajectory_type,'moment')
    bool_position = 0; % {0,1}
    bool_orientation = 0; % {0,1}
    bool_force = 0; % {0,1}
    bool_moment = 1; % {0,1}
end
