function load_casadi_library()
% This function automatically adds the downloaded Casadi 3.6.3 folder in
% 'libraries/' to the Matlab path whether you are in Linux, Mac or Windows.

% Find path of directory where the Casadi folder should be located
path = mfilename('fullpath');
directoryname = [fileparts(path),'/../../libraries/'];

% Set name of Casadi folder depending on operating system
if ismac
    foldername = 'casadi-3.6.3-osx64-matlab2018b';
elseif isunix
    foldername = 'casadi-3.6.3-linux64-matlab2018b';
elseif ispc
    % foldername = 'casadi-3.6.3-windows64-matlab2018b';
    foldername = 'casadi-windows-matlabR2016a-v3.5.5';
else
    error('Your Operating System does not support Casadi, check https://web.casadi.org/get/')
end

% Add Casadi folder to the Matlab path
if isfolder([directoryname,foldername])
    addpath([directoryname,foldername]);
else
    error(['Cannot find Casadi in this path: ', directoryname,foldername])
end

