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
    
    data{i}.t       = data_file{i}.data(:, find(strcmp(data_file{i}.textdata, header.t)));
    
    Px              = data_file{i}.data(:, find(strcmp(data_file{i}.textdata, header.Px)));
    Py              = data_file{i}.data(:, find(strcmp(data_file{i}.textdata, header.Py)));
    Pz              = data_file{i}.data(:, find(strcmp(data_file{i}.textdata, header.Pz)));
    
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
















