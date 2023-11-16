function plot_contour_edges(position_tcp, N)
% Estimate the pose of the contour
contour_edge = load('data/stl/contour/contour_edge');
contour_edge = contour_edge/1000; % convert mm to m
average_contour_edge = mean(contour_edge,1);
delta_contour_edge = contour_edge - average_contour_edge;
average_position_tcp = mean(position_tcp,1);
delta_position_tcp = position_tcp - average_position_tcp;

% Calculate the spatial alignment
S = delta_contour_edge'*delta_position_tcp;
[U,~,V] = svd(S);
ROTATION = V*U';
if det(ROTATION) < 0
    ROTATION = V*[1,0,0;0,1,0;0,0,-1]*U';
end
TRANSLATION = (average_position_tcp' - ROTATION*average_contour_edge')';

% Plot the contour
contour_color = [200 200 200]/255;
plot_stl('data/stl/contour/contour.STL',ROTATION,TRANSLATION,0.001,contour_color,0.5)

p_o_orig = load('data/stl/contour/contour_edge_o');
p_y_orig = load('data/stl/contour/contour_edge_y');
p_z_orig = load('data/stl/contour/contour_edge_z');
for j = 1 : N
    p_o_orig(j,:) = ROTATION*p_o_orig(j,:)'+TRANSLATION';
    p_y_orig(j,:) = ROTATION*p_y_orig(j,:)'+TRANSLATION';
    p_z_orig(j,:) = ROTATION*p_z_orig(j,:)'+TRANSLATION';
end
plot3(p_o_orig(:,1),p_o_orig(:,2),p_o_orig(:,3),'k','linewidth',1.5)
plot3(p_y_orig(:,1),p_y_orig(:,2),p_y_orig(:,3),'k','linewidth',1.5)
plot3(p_z_orig(:,1),p_z_orig(:,2),p_z_orig(:,3),'k','linewidth',1.5)
plot3([p_o_orig(1,1),p_y_orig(1,1)],[p_o_orig(1,2),p_y_orig(1,2)],[p_o_orig(1,3),p_y_orig(1,3)],'k','linewidth',1.5)
plot3([p_o_orig(1,1),p_z_orig(1,1)],[p_o_orig(1,2),p_z_orig(1,2)],[p_o_orig(1,3),p_z_orig(1,3)],'k','linewidth',1.5)
plot3([p_o_orig(end,1),p_y_orig(end,1)],[p_o_orig(end,2),p_y_orig(end,2)],[p_o_orig(end,3),p_y_orig(end,3)],'k','linewidth',1.5)
plot3([p_o_orig(end,1),p_z_orig(end,1)],[p_o_orig(end,2),p_z_orig(end,2)],[p_o_orig(end,3),p_z_orig(end,3)],'k','linewidth',1.5)
plot3([p_y_orig(1,1),p_z_orig(1,1)],[p_y_orig(1,2),p_z_orig(1,2)],[p_y_orig(1,3),p_z_orig(1,3)],'k','linewidth',1.5)
plot3([p_y_orig(end,1),p_z_orig(end,1)],[p_y_orig(end,2),p_z_orig(end,2)],[p_y_orig(end,3),p_z_orig(end,3)],'k','linewidth',1.5)
end