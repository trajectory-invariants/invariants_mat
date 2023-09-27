function data = load_all_trials_contour(path_to_data)
% This function sets the pasth to the data and load the relevant data
% through a function called data_structure based on the defined headers.
% 
% Input:
%     path_to_data:     path to the directory of data                       [string]
% 
% Output:
%     data:             data is loaded and incldues time, pose, and wrench  [cell]

%% Headers for the .dat files

header.t = 'TimeStamp';                                 % Time
header.Px = 'read_tf.outport_pose.0';                   % X-axis of the position
header.Py = 'read_tf.outport_pose.1';                   % Y-axis of the position
header.Pz = 'read_tf.outport_pose.2';                   % Z-axis of the position
header.rw = 'read_tf.outport_pose.3';                   % Scalar part of the quaternion
header.rx = 'read_tf.outport_pose.4';                   % X-axis of the vector part of the quaternion
header.ry = 'read_tf.outport_pose.5';                   % Y-axis of the vector part of the quaternion
header.rz = 'read_tf.outport_pose.6';                   % Z-axis of the vector part of the quaternion
header.Fx = 'jr3.sensor_0.wrench_filter_0.force.x';     % X-axis of the force
header.Fy = 'jr3.sensor_0.wrench_filter_0.force.y';     % Y-axis of the force
header.Fz = 'jr3.sensor_0.wrench_filter_0.force.z';     % Z-axis of the force
header.Tx = 'jr3.sensor_0.wrench_filter_0.torque.x';    % X-axis of the torque
header.Ty = 'jr3.sensor_0.wrench_filter_0.torque.y';    % Y-axis of the torque
header.Tz = 'jr3.sensor_0.wrench_filter_0.torque.z';    % Z-axis of the torque

%% Load the data

data = data_structure(path_to_data,header);