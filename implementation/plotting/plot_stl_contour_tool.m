function plot_stl_contour_tool(rotation_tcp, position_tcp, trial)
sensor_id = configurations(); % number identifying the location and orientation of the force sensor {lc} and motion tracker {tr} on the tool

% Colors
tracker_color = [30 30 30]/255;
sensor_color = [0 0 205]/255;
part1_color = [60 60 60]/255;
part2_color = [80 80 80]/255;
part3_color = [100 100 100]/255;
part4_color = [120 120 120]/255;
bearings_color = [0 0 0]/255;

% Choose some samples to plot by stl files
k_1 = 1;
nb_rigid_bodies = 3;
rotation_tcp_k = zeros(3,3,nb_rigid_bodies);
position_tcp_k = zeros(nb_rigid_bodies,3);
for j = round(linspace(1,N,nb_rigid_bodies))
    rotation_tcp_k(:,:,k_1) = rotation_tcp(:,:,j);
    position_tcp_k(k_1,:)   = position_tcp(j,:);
    k_1                     = k_1+1;
end

for j = 1 : k_1-1
    plot_stl(['data/stl/contour/configuration-',sensor_id(trial),'/tool-',sensor_id(trial),'-bearings.STL'],rotation_tcp_k(:,:,j),position_tcp_k(j,:),0.001,bearings_color,0.8);
    plot_stl(['data/stl/contour/configuration-',sensor_id(trial),'/tool-',sensor_id(trial),'-print4.STL'],rotation_tcp_k(:,:,j),position_tcp_k(j,:),0.001,part4_color,0.8);
    plot_stl(['data/stl/contour/configuration-',sensor_id(trial),'/tool-',sensor_id(trial),'-print1.STL'],rotation_tcp_k(:,:,j),position_tcp_k(j,:),0.001,part1_color,0.8);
    plot_stl(['data/stl/contour/configuration-',sensor_id(trial),'/tool-',sensor_id(trial),'-print2.STL'],rotation_tcp_k(:,:,j),position_tcp_k(j,:),0.001,part2_color,0.8);
    plot_stl(['data/stl/contour/configuration-',sensor_id(trial),'/tool-',sensor_id(trial),'-print3.STL'],rotation_tcp_k(:,:,j),position_tcp_k(j,:),0.001,part3_color,0.8);
    plot_stl(['data/stl/contour/configuration-',sensor_id(trial),'/tool-',sensor_id(trial),'-sensor.STL'],rotation_tcp_k(:,:,j),position_tcp_k(j,:),0.001,sensor_color,0.8);
    plot_stl(['data/stl/contour/configuration-',sensor_id(trial),'/tool-',sensor_id(trial),'-tracker.STL'],rotation_tcp_k(:,:,j),position_tcp_k(j,:),0.001,tracker_color,0.8);
end
end