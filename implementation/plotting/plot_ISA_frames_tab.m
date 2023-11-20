function plot_ISA_frames_tab(T_isa, pose_tcp, trial, tab1)

% Plot settings
axes(tab1); hold on;
view_fig = [-50,10];
axis_font_size = 12;
label_font_size = 18;
set(groot,'defaultAxesTickLabelInterpreter','latex');
view(view_fig);

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
plot_contour_object(position_tcp, path_to_stl_data);

% Plot tool at correct position and orientation in world
plot_stl_contour_tool(rotation_tcp, position_tcp, trial, path_to_stl_data);


%% Plot ISA frames and screw axes


% Plot ISA frames
arrow_length_isa = 0.1;
arrow_length_normal_binormal = 0.05;
arrow_width_isa = 0.0015;
arrow_width_normal_binormal = 0.00075;
steps_isa = 2;
plot_isa_frames(T_isa,arrow_length_isa,arrow_length_normal_binormal,arrow_width_isa,arrow_width_normal_binormal,steps_isa,darkred,green,blue)

% Plot ISA axes
length_screw_back = 0.15;
length_screw_front = 0.15;
width_axis = 0.5;
plots_screw_axes(T_isa,length_screw_back,length_screw_front,grey,width_axis,steps_isa);

%% Plot styling

set(gcf,'units','normalized','outerposition',[0 0.04 0.7 0.74]);
set(gca,'FontSize',axis_font_size)
axis equal; view(view_fig); grid on; box on;
xlabel('$x\ [\mathrm{m}]$','Interpreter','LaTex','FontSize',label_font_size)
ylabel('$y\ [\mathrm{m}]$','Interpreter','LaTex','FontSize',label_font_size)
zlabel('$z\ [\mathrm{m}]$','Interpreter','LaTex','FontSize',label_font_size)

end
