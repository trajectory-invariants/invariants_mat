function plot_contour_object_movie(position_tcp, path_to_stl_data)

N = size(position_tcp,1);

% Load contour edge
contour_edge_struct = load([path_to_stl_data,'contour_edge']);
contour_edge = contour_edge_struct.contour_edge/1000; % convert mm to m

% Calculate the spatial alignment between the contour edge and traveled
% trajectory to estimate where the contour is (purely for visualization)
average_contour_edge = mean(contour_edge,1);
delta_contour_edge = contour_edge - average_contour_edge;
average_position_tcp = mean(position_tcp,1);
delta_position_tcp = position_tcp - average_position_tcp;
S = delta_contour_edge'*delta_position_tcp;
[U,~,V] = svd(S);
ROTATION = V*U';
if det(ROTATION) < 0
    ROTATION = V*[1,0,0;0,1,0;0,0,-1]*U';
end
TRANSLATION = (average_position_tcp' - ROTATION*average_contour_edge')';

% Plot the contour object based on stl file
contour_color = [230 230 230]/255;
plot_stl([path_to_stl_data,'contour.STL'],ROTATION,TRANSLATION,0.001,contour_color,1)
material dull

end