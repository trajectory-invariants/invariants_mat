function check_input(settings_analysis)
% This function checks whether the selected combination in trajectory type,
% viewpoint and reference point are supported by the toolbox

%% Supported combinations in:
% trajectory type + viewpoint +  reference point

% {motion}        +   {world}   +   {tracker} or {tool_point}
% {motion}        +   {body}    +   {middle_contour} 
% {wrench}        +   {world}   +   {force_sensor}
% {wrench}        +   {body}    +   {tracker} or {tool_point} 

type = settings_analysis.trajectory_type;
viewpoint = settings_analysis.viewpoint;
referencepoint = settings_analysis.referencepoint;

%%
bool = zeros(1,11);
bool(1) = strcmp(type,'pose') && strcmp(viewpoint,'world') && strcmp(referencepoint,'tracker');
bool(2) = strcmp(type,'pose') && strcmp(viewpoint,'world') && strcmp(referencepoint,'tool_point');
bool(3) = strcmp(type,'pose') && strcmp(viewpoint,'body') && strcmp(referencepoint,'middle_contour');
bool(4) = strcmp(type,'wrench') && strcmp(viewpoint,'world') && strcmp(referencepoint,'force_sensor');
bool(5) = strcmp(type,'wrench') && strcmp(viewpoint,'body') && strcmp(referencepoint,'tracker');
bool(6) = strcmp(type,'wrench') && strcmp(viewpoint,'body') && strcmp(referencepoint,'tool_point');

if ~sum(bool)
    error('Selected configuration is not supported. Choose another trajectory, viewpoint or referencepoint')
end

end