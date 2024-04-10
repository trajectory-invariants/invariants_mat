function plot_stl_contour_tool_movie(rotation_tcp, position_tcp, trial, path_to_stl_data, index)

sensor_id = configurations(); % number identifying the location and orientation of the force sensor {lc} and motion tracker {tr} on the tool

% Colors
part4_color = [150 150 150]/255;
bearings_color = [0 0 0]/255;

% Plot all parts of the tool object
if trial >= 1 && trial <= 12
    plot_stl([path_to_stl_data,'configuration-',sensor_id(trial),'/tool-',sensor_id(trial),'-bearings.STL'],rotation_tcp(:,:,index),position_tcp(index,:),0.001,bearings_color,1);
    material dull
    hold on
    plot_stl([path_to_stl_data,'configuration-',sensor_id(trial),'/tool-',sensor_id(trial),'-print4.STL'],rotation_tcp(:,:,index),position_tcp(index,:),0.001,part4_color,1);
    material dull
    hold on
    %plot_stl([path_to_stl_data,'configuration-',sensor_id(trial),'/tool-',sensor_id(trial),'-print1.STL'],rotation_tcp_k(:,:,j),position_tcp_k(j,:),0.001,part1_color,0.8);
    %plot_stl([path_to_stl_data,'configuration-',sensor_id(trial),'/tool-',sensor_id(trial),'-print2.STL'],rotation_tcp_k(:,:,j),position_tcp_k(j,:),0.001,part2_color,0.8);
    %plot_stl([path_to_stl_data,'configuration-',sensor_id(trial),'/tool-',sensor_id(trial),'-print3.STL'],rotation_tcp_k(:,:,j),position_tcp_k(j,:),0.001,part3_color,0.8);
    %plot_stl([path_to_stl_data,'configuration-',sensor_id(trial),'/tool-',sensor_id(trial),'-sensor.STL'],rotation_tcp_k(:,:,j),position_tcp_k(j,:),0.001,sensor_color,0.8);
    %plot_stl([path_to_stl_data,'configuration-',sensor_id(trial),'/tool-',sensor_id(trial),'-tracker.STL'],rotation_tcp_k(:,:,j),position_tcp_k(j,:),0.001,tracker_color,0.8);
end
end