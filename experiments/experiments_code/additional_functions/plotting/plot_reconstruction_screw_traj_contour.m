function plot_reconstruction_screw_traj_contour(measured_pose,measured_wrench,T_global,recons_pose,T_isa_pose,recons_wrench,T_isa_wrench,trial)

% Plotting parameters

% number identifying the location and orientation of the force sensor {lc} and motion tracker {tr} on the tool
sensor_id = configurations();
nb_rigid_bodies = 5;

% Colors
part4_color = [120 120 120]/255;
bearings_color = [0 0 0]/255;
contour_color = [200 200 200]/255;
darkred = [0.6350 0.0780 0.1840];
blue = [0 0.4470 0.7410];
grey = [0.8  0.8 0.8];

N_des = 101;
N = 101;

%% Measurements
% Fetch data to approximate the pose of the contour and tool
rotation_tcp_meas = measured_pose(1:3,1:3,:);
position_tcp_meas = squeeze(measured_pose(1:3,4,:))';

% Choose some samples to plot by stl files
k_1 = 1;
for j = round(linspace(1,N_des,nb_rigid_bodies))
    rotation_tcp_k_meas(:,:,k_1) = rotation_tcp_meas(1:3,1:3,j);
    position_tcp_k_meas(k_1,:)   = position_tcp_meas(j,:);
    k_1                     = k_1+1;
end

% Estimate the pose of the contour
load('../data/stl/contour/contour_edge');
contour_edge = contour_edge/1000; % convert mm to m
average_contour_edge = mean(contour_edge,1);
delta_contour_edge = contour_edge - average_contour_edge;
average_position_tcp = mean(position_tcp_meas,1);
delta_position_tcp = position_tcp_meas - average_position_tcp;

% Calculate the spatial alignment
S = delta_contour_edge'*delta_position_tcp;
[U,~,V] = svd(S);
ROTATION = V*U';
if det(ROTATION) < 0
    ROTATION = V*[1,0,0;0,1,0;0,0,-1]*U';
end
TRANSLATION = (average_position_tcp' - ROTATION*average_contour_edge')';

%% Plot measured trajectory
figure('Color',[1 1 1]); hold on; axis equal;

% Plot the contour
plot_stl('../data/stl/contour/contour.STL',ROTATION,TRANSLATION,0.001,contour_color,0.5)
view(1,2);axis('equal')

% Plot the follower
if ~strcmp(trial,'ref') % not the reference case
    for j = 1 : k_1-1
        plot_stl(['../data/stl/contour/configuration-',sensor_id(trial),'/tool-',sensor_id(trial),'-bearings.STL'],rotation_tcp_k_meas(:,:,j),position_tcp_k_meas(j,:),0.001,bearings_color,1);
        plot_stl(['../data/stl/contour/configuration-',sensor_id(trial),'/tool-',sensor_id(trial),'-print4.STL'],rotation_tcp_k_meas(:,:,j),position_tcp_k_meas(j,:),0.001,part4_color,1);
    end
end

% Plot position
position_traj = squeeze(measured_pose(1:3,4,:))';
plot3(position_traj(:,1),position_traj(:,2),position_traj(:,3),'b','LineWidth',2)

% Plot force
force_traj = measured_wrench(:,1:3);
for i=1:5:N
    mArrow3(position_tcp_meas(i,:),position_tcp_meas(i,:)+0.1*force_traj(i,:)/norm(force_traj(i,:)),'color',darkred,'stemWidth',0.0005*5);
end

%% Reconstructed trajectory
rotation_tcp = recons_pose(1:3,1:3,:);
position_tcp = squeeze(recons_pose(1:3,4,:))';

% Choose some samples to plot by stl files
k_1 = 1;
for j = round(linspace(1,N_des,nb_rigid_bodies))
    rotation_tcp_k(:,:,k_1) = rotation_tcp(1:3,1:3,j);
    position_tcp_k(k_1,:)   = position_tcp(j,:);
    k_1                     = k_1+1;
end

% Plot stl files in correct position and orientation
if ~strcmp(trial,'ref') % not the reference case
    for j = 1 : k_1-1
        if j < 2
            plot_stl(['../data/stl/contour/configuration-',sensor_id(trial),'/tool-',sensor_id(trial),'-bearings.STL'],rotation_tcp_k(:,:,j),position_tcp_k(j,:),0.001,bearings_color,0);
            plot_stl(['../data/stl/contour/configuration-',sensor_id(trial),'/tool-',sensor_id(trial),'-print4.STL'],rotation_tcp_k(:,:,j),position_tcp_k(j,:),0.001,part4_color,0);
        else
            bearings_color = [120 120 120]/255;
            part4_color = [180 180 180]/255;
            plot_stl(['../data/stl/contour/configuration-',sensor_id(trial),'/tool-',sensor_id(trial),'-bearings.STL'],rotation_tcp_k(:,:,j),position_tcp_k(j,:),0.001,bearings_color,1);
            plot_stl(['../data/stl/contour/configuration-',sensor_id(trial),'/tool-',sensor_id(trial),'-print4.STL'],rotation_tcp_k(:,:,j),position_tcp_k(j,:),0.001,part4_color,1);
        end
    end
end

% positions
position_traj = squeeze(recons_pose(1:3,4,:))';
plot3(position_traj(:,1),position_traj(:,2),position_traj(:,3),'b','LineWidth',2)

% force
force_traj = recons_wrench(1:3,:)';
for i=1:5:N
    mArrow3(position_tcp(i,:),position_tcp(i,:)+0.1*force_traj(i,:)/norm(force_traj(i,:)),'color',darkred,'stemWidth',0.0005*5,'facealpha',0.25);
end

%% Initial ISA frame motion
p_isa = squeeze(T_isa_pose(1:3,4,1))';
x_isa = squeeze(T_isa_pose(1:3,1,1))';
y_isa = squeeze(T_isa_pose(1:3,2,1))';
z_isa = squeeze(T_isa_pose(1:3,3,1))';

arrow_length_isa = 0.1;
arrow_width_isa = 0.0025;
mArrow3(p_isa,p_isa+arrow_length_isa*x_isa,'color',blue,'stemWidth',arrow_width_isa);
mArrow3(p_isa,p_isa+arrow_length_isa*y_isa*0.5,'color',blue,'stemWidth',arrow_width_isa*0.5);
mArrow3(p_isa,p_isa+arrow_length_isa*z_isa*0.5,'color',blue,'stemWidth',arrow_width_isa*0.5);

length_screw = 0.20;
screw_start = p_isa(1,:) - x_isa(1,:)*length_screw*0.5;
screw_end = p_isa(1,:) + x_isa(1,:)*length_screw;
plot3([screw_start(1) screw_end(1)],[screw_start(2) screw_end(2)],[screw_start(3) screw_end(3)],'Color',grey,'LineWidth',2)

%% Initial ISA frame wrench
p_virfs_w = ((position_tcp(1,:)+position_tcp(end,:))/2)';
%[R_virfs,~] = make_artificial_variations(eye(3),[0,0,0],trial);
T_virfs = compose_pose_matrix(T_global(1:3,1:3),p_virfs_w');
T_isa_wrench_0 = T_virfs*inverse_pose(T_global)*T_isa_wrench(:,:,1);

p_isa = squeeze(T_isa_wrench_0(1:3,4,1))';
x_isa = squeeze(T_isa_wrench_0(1:3,1,1))';
y_isa = squeeze(T_isa_wrench_0(1:3,2,1))';
z_isa = squeeze(T_isa_wrench_0(1:3,3,1))';

arrow_length_isa = 0.1;
arrow_width_isa = 0.0025;
mArrow3(p_isa,p_isa+arrow_length_isa*x_isa,'color',darkred,'stemWidth',arrow_width_isa);
mArrow3(p_isa,p_isa+arrow_length_isa*y_isa*0.5,'color',darkred,'stemWidth',arrow_width_isa*0.5);
mArrow3(p_isa,p_isa+arrow_length_isa*z_isa*0.5,'color',darkred,'stemWidth',arrow_width_isa*0.5);

length_screw = 0.20;
screw_start = p_isa(1,:) - x_isa(1,:)*length_screw*0.5;
screw_end = p_isa(1,:) + x_isa(1,:)*length_screw;
plot3([screw_start(1) screw_end(1)],[screw_start(2) screw_end(2)],[screw_start(3) screw_end(3)],'Color',grey,'LineWidth',2)


%% Plotting parameters 
label_font_size = 25;
set(gcf,'units','normalized','outerposition',[0 0.04 0.7 0.74]);
set(groot,'defaultAxesTickLabelInterpreter','latex');
set(gca,'FontSize',20)
hold on; axis equal; grid on; box on;

xlabel('$x\ [\mathrm{m}]$','Interpreter','LaTex','FontSize',label_font_size)
ylabel('$y\ [\mathrm{m}]$','Interpreter','LaTex','FontSize',label_font_size)
zlabel('$z\ [\mathrm{m}]$','Interpreter','LaTex','FontSize',label_font_size)

% xlim([-0.4 0.1])
% ylim([1.3 2.1])
% zlim([-0.6 -0.2])
view([99,23])

