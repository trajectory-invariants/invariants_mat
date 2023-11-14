function plot_stl(filename,R,location,scale,filecolor,filealpha,edgealpha)
% Plot a 3D stl file as a rigid body in a figure based on
% a rotation matrix

% Input:
%   filename                string but ended with .stl => filename.stl
%   R                       [3x3]
%   location                [3x1]
%   scale                   [1]
%   filecolor               [1x3] for rgb or a color
%   filealpha               [1]

%%



% if strcmp(filename(18:end),'contour.STL')*0
%     [TR,~,~,~]  = stlread(filename);
%     vertices    = TR.Points;
%     faces       = TR.ConnectivityList;
%
%     if nargin < 7
%         edgealpha = filealpha*0.2;
%     end
%     for j = 1 : length(vertices)
%         vertices_transformed(j,:) = R*vertices(j,:)'.*scale+location';
%     end
%
%     patch('Faces',faces,'Vertices',vertices_transformed,'FaceColor',filecolor,...
%         'EdgeColor','k','FaceAlpha',filealpha,'EdgeAlpha',edgealpha); % add edgealpha as an input parameter
% else
[TR,~,~,~]  = stlread(filename);
vertices    = TR.Points;
faces       = TR.ConnectivityList;

for j = 1 : length(vertices)
    vertices_transformed(j,:) = R*vertices(j,:)'.*scale+location';
end
if nargin < 7
    edgealpha = filealpha*0.5;
end

patch('Faces',faces,'Vertices',vertices_transformed,'FaceColor',filecolor,...
    'EdgeColor','none','FaceAlpha',filealpha,'EdgeAlpha',edgealpha);
% end

