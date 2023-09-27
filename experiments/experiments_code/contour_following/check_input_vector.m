function check_input_vector(type,viewpoint,referencepoint)
% This function checks whether the selected combination in trajectory type,
% viewpoint and reference point are supported by the toolbox

%% Supported combinations in:
% trajectory type + viewpoint +  reference point

% {orientation}   +   {world}   +   {tracker} or {tool_point}
% {orientation}   +   {body}    +   {tracker} or {tool_point}
% {force}         +   {world}   +   {tracker} or {tool_point} or {force_sensor}
% {force}         +   {body}    +   {tracker} or {tool_point} 
% {position}      +   {world}   +   {tracker} or {tool_point}
% {position}      +   {body}    +   {tool_point} or {tracker} or {middle_contour}
% {moment}        +   {world}   +   {tracker} or {tool_point} or {force_sensor}
% {moment}        +   {body}    +   {tracker} or {tool_point}

%%
bool = zeros(1,11);
bool(1) = strcmp(type,'orientation') && (strcmp(referencepoint,'tracker') || strcmp(referencepoint,'tool_point') ...
    || strcmp(referencepoint,'middle_contour'));
bool(2) = strcmp(type,'force') && strcmp(viewpoint,'world') && ...
    (strcmp(referencepoint,'tracker') || strcmp(referencepoint,'tool_point') || strcmp(referencepoint,'force_sensor'));
bool(3) = strcmp(type,'force') && strcmp(viewpoint,'body') && ...
    (strcmp(referencepoint,'tracker') || strcmp(referencepoint,'tool_point'));
bool(4) = strcmp(type,'position') && strcmp(viewpoint,'world') && strcmp(referencepoint,'tracker');
bool(5) = strcmp(type,'position') && strcmp(viewpoint,'world') && strcmp(referencepoint,'tool_point');
bool(6) = strcmp(type,'position') && strcmp(viewpoint,'body') && strcmp(referencepoint,'middle_contour');
bool(7) = strcmp(type,'position') && strcmp(viewpoint,'body') && strcmp(referencepoint,'tool_point');
bool(8) = strcmp(type,'position') && strcmp(viewpoint,'body') && strcmp(referencepoint,'tracker');
bool(9) = strcmp(type,'moment') && strcmp(viewpoint,'world') && strcmp(referencepoint,'tracker');
bool(10) = strcmp(type,'moment') && strcmp(viewpoint,'world') && strcmp(referencepoint,'tool_point');
bool(11) = strcmp(type,'moment') && strcmp(viewpoint,'world') && strcmp(referencepoint,'force_sensor');
bool(12) = strcmp(type,'moment') && strcmp(viewpoint,'body') && strcmp(referencepoint,'tracker');
bool(13) = strcmp(type,'moment') && strcmp(viewpoint,'body') && strcmp(referencepoint,'tool_point');

if ~sum(bool)
    error('Selected configuration is not supported. Choose another trajectory, viewpoint or referencepoint')
end

end