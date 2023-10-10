function data = load_trials_in_folder(path_to_data)
% This function sets the path to the data and load the relevant data
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


function data = data_structure(testfiledir,header)
% This function converts the structure of the data in such a way that all
% the data including time, position, quaternion, rotation, force, and
% torque are represented by a cell.
% 
% Input:
%     testfiledir:  directory address which containts .dat files    [string]
%     header:       this header must be defined in the main code    [structure]
% 
% Output:
%     data:         all trials stored as a cell                     [cell]
%                   data.pos
%                   data.quat
%                   data.rotm
%                   data.force
%                   data.torque

%% Load .dat data and covert it to cell

data_file           = load_data(testfiledir);
nfiles              = size(data_file,1);
data                = cell(nfiles,1);

for i = 1 : nfiles
    
    data{i}.t       = data_file{i}.data(:, strcmp(data_file{i}.textdata, header.t));
    
    Px              = data_file{i}.data(:, strcmp(data_file{i}.textdata, header.Px));
    Py              = data_file{i}.data(:, strcmp(data_file{i}.textdata, header.Py));
    Pz              = data_file{i}.data(:, strcmp(data_file{i}.textdata, header.Pz));
    
    rw              = data_file{i}.data(:, find(strcmp(data_file{i}.textdata, header.rw)));
    rx              = data_file{i}.data(:, find(strcmp(data_file{i}.textdata, header.rx)));
    ry              = data_file{i}.data(:, find(strcmp(data_file{i}.textdata, header.ry)));
    rz              = data_file{i}.data(:, find(strcmp(data_file{i}.textdata, header.rz)));
    
    Fx              = data_file{i}.data(:, find(strcmp(data_file{i}.textdata, header.Fx)));
    Fy              = data_file{i}.data(:, find(strcmp(data_file{i}.textdata, header.Fy)));
    Fz              = data_file{i}.data(:, find(strcmp(data_file{i}.textdata, header.Fz)));
    
    Tx              = data_file{i}.data(:, find(strcmp(data_file{i}.textdata, header.Tx)));
    Ty              = data_file{i}.data(:, find(strcmp(data_file{i}.textdata, header.Ty)));
    Tz              = data_file{i}.data(:, find(strcmp(data_file{i}.textdata, header.Tz)));
    
    
    data{i}.pos     = cat(2,Px,Py,Pz);
    data{i}.quat    = cat(2,rw,rx,ry,rz);
    data{i}.rotm    = quat2rotm(data{i}.quat );
    data{i}.force   = cat(2,Fx,Fy,Fz);
    data{i}.torque  = cat(2,Tx,Ty,Tz);
end
















