function plot_ISA_frame(T_isa, pose_tcp, trial, index, view_movie)

% Plot settings
axis_font_size = 12;
label_font_size = 18;
set(groot,'defaultAxesTickLabelInterpreter','latex');

% Colors
darkred = [0.6350 0.0780 0.1840];
green = [0.4660 0.6740 0.1880];
blue = [0 0.4470 0.7410];
grey = [0.8  0.8 0.8];

%% Revert artificial variations

% Extend to 4x4 matrix
T_isa = extend_homogeneous_matrix(T_isa);
[rotation_tcp,position_tcp] = reverse_artificial_variations(pose_tcp,trial);
[~,~,T_isa] = reverse_artificial_variations(T_isa,trial);

%% Plot 3D objects

% Plot contour at correct position in world
path_to_stl_data = 'data/contour_following/stl/';
plot_contour_object_movie(position_tcp, path_to_stl_data);
hold on;

% Plot tool at correct position and orientation in world
plot_stl_contour_tool_movie(rotation_tcp, position_tcp, trial, path_to_stl_data, index);
hold on;

%% Plot ISA frames and screw axes

% Plot ISA frames
arrow_length_isa = 0.1;
arrow_length_normal_binormal = 0.05;
arrow_width_isa = 0.0025;
arrow_width_normal_binormal = 0.0015;
steps_isa = 2;
plot_isa_frame_movie(T_isa(:,:,index),arrow_length_isa,arrow_length_normal_binormal,arrow_width_isa,arrow_width_normal_binormal,steps_isa,darkred,green,blue)
hold on;

% Plot ISA axes
length_screw_back = 0.15;
length_screw_front = 0.15;
width_axis = 1;
plots_screw_axes_movie(T_isa(:,:,index),length_screw_back,length_screw_front,grey,width_axis,steps_isa);
hold on;

% Plot trajectory origin ISA
pos_isa = squeeze(T_isa(1:3,4,:));
plot3(pos_isa(1,1:index),pos_isa(2,1:index),pos_isa(3,1:index),'k:','LineWidth',1.5)
hold on;

%% Plot styling

set(gca,'FontSize',axis_font_size)
axis equal; grid on; box on;
xlabel('$x\ [\mathrm{m}]$','Interpreter','LaTex','FontSize',label_font_size)
ylabel('$y\ [\mathrm{m}]$','Interpreter','LaTex','FontSize',label_font_size)
zlabel('$z\ [\mathrm{m}]$','Interpreter','LaTex','FontSize',label_font_size)
% view(160,15)
% view(90,90)
view(view_movie)

xlim([-0.4 0.2])
ylim([1.3 1.7])
zlim([-1.12 -0.75])
% axis off
% grid off

light;
drawnow
100;


end
