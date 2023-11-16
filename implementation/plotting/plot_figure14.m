function plot_figure14(T_isa_data, pose, trial, referencepoint,view_angles)

figure('Name',strcat(['ISA frames of trial ',num2str(trial)]),'Color',[1 1 1],'NumberTitle','off'); hold on; axis equal;
set(groot,'defaultAxesTickLabelInterpreter','latex');

% Properties plot
axis_font_size = 35;
label_font_size = 38;
%view_fig = [34,21];

% Parameters stl plots
nb_rigid_bodies = 3;

% Parameters ISA plots
arrow_length_isa = 0.075;
arrow_length_normal_binormal = 0.05;
arrow_width_isa = 0.0010;
arrow_width_normal_binormal = 0.00075;
steps_isa = 20;

length_screw_back = 0.075;
length_screw_front = 0.10;
width_axis = 1;

% Colors
% tracker_color = [30 30 30]/255;
% sensor_color = [0 0 205]/255;
% part1_color = [60 60 60]/255;
% part2_color = [80 80 80]/255;
% part3_color = [100 100 100]/255;
% part4_color = [120 120 120]/255;
% bearings_color = [0 0 0]/255;
% contour_color = [200 200 200]/255;
darkred = [0.6350 0.0780 0.1840];
%red = [0.8500 0.3250 0.0980];
green = [0.4660 0.6740 0.1880];
blue = [0 0.4470 0.7410];
grey = [0.8  0.8 0.8];
darkgrey = [0.5  0.5 0.5];

%% Plot 3D objects

% Fetch measured position data, necessary to plot objects in right location
N = size(T_isa_data,3);
[~,~,position_tcp,rotation_tcp,~,~,~,~,~,~,~,~,~,~] = contour_preprocess_data(N,'world','dimless_arclength','tool_point',trial,trial,'peg','real','./data/');

% Plot peg stl
for j = round(linspace(1,N,nb_rigid_bodies))
    plot_stl('data/stl/peg/peg.STL',rotation_tcp(:,:,j),position_tcp(j,:),0.001,[0 0 0],0.05);
end
view(1,2); axis('equal'); % the "view" command is required for some reason to get axis equal to work

% Plot hole stl
plot_stl('data/stl/peg/hole.STL',eye(3),position_tcp(end,:),0.001,blue,0.07);

%% Plot ISA frames and screw axes

% Extend to 4x4 matrix
T_isa_data = extend_homogeneous_matrix(T_isa_data);

% Revert artificial transformation of force sensor (?)
if strcmp(referencepoint,'force_sensor')
    for k = 1:N
        T_isa_data(:,:,k) = pose(:,:,k)*T_isa_data(:,:,k); 
    end
end

% Plot ISA axes
plots_screw_axes(T_isa_data,length_screw_back,length_screw_front,darkred,width_axis,steps_isa);

% Plot ISA frames
plot_isa_frames(T_isa_data,arrow_length_isa,arrow_length_normal_binormal,arrow_width_isa,arrow_width_normal_binormal,steps_isa,darkred,green,blue)

%% Plot styling
set(gcf,'units','normalized','outerposition',[0 0.04 0.7 0.74]);
set(gca,'FontSize',axis_font_size)
axis equal; view(view_angles); grid on; box on;
xlabel('$x\ [\mathrm{m}]$','Interpreter','LaTex','FontSize',label_font_size)
ylabel('$y\ [\mathrm{m}]$','Interpreter','LaTex','FontSize',label_font_size)
zlabel('$z\ [\mathrm{m}]$','Interpreter','LaTex','FontSize',label_font_size)
