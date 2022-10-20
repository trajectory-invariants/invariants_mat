function data = load_data(testfiledir)
% Load all .dat files in the given directory and store the data as a cell
%
% Input:
%     testfiledir: directory path which contains .dat files     [string]
%
% Output:
%     data: all trials stored as a cell                         [cell]

%%
matfiles = dir(fullfile(testfiledir, '*.dat'));
nfiles = length(matfiles);
data  = cell(nfiles,1);
for i = 1 : nfiles
    data{i} = importdata( fullfile(testfiledir, matfiles(i).name) );
end