function plot_transformed_screw_traj_contour(measured_pose,measured_wrench,measured_pose_transf,measured_wrench_transf,trial)

% Plotting parameters

% number identifying the location and orientation of the force sensor {lc} and motion tracker {tr} on the tool
sensor_id = configurations();
nb_rigid_bodies = 5;

% colors
part4_color = [120 120 120]/255;
bearings_color = [0 0 0]/255;
contour_color = [200 200 200]/255;
darkred = [0.6350 0.0780 0.1840];

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

%% Plotting the original measurements

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

% Plot the position trajectory
plot3(position_tcp_meas(:,1),position_tcp_meas(:,2),position_tcp_meas(:,3),'b','LineWidth',2)

% Plot the force trajectory
force_traj = measured_wrench(:,1:3);
for i=1:5:N
    mArrow3(position_tcp_meas(i,:),position_tcp_meas(i,:)+0.1*force_traj(i,:)/norm(force_traj(i,:)),'color',darkred,'stemWidth',0.0005*5);
end

%% Plotting the transformed measurements

rotation_tcp = measured_pose_transf(1:3,1:3,:);
position_tcp = squeeze(measured_pose_transf(1:3,4,:))';

% Choose some samples to plot by stl files
k_1 = 1;
for j = round(linspace(1,N_des,nb_rigid_bodies))
    rotation_tcp_k(:,:,k_1) = rotation_tcp(1:3,1:3,j);
    position_tcp_k(k_1,:)   = position_tcp(j,:);
    k_1                     = k_1+1;
end

% Plot followers
if ~strcmp(trial,'ref') % not the reference case
    for j = 1 : k_1-1
        plot_stl(['../data/stl/contour/configuration-',sensor_id(trial),'/tool-',sensor_id(trial),'-bearings.STL'],rotation_tcp_k(:,:,j),position_tcp_k(j,:),0.001,bearings_color,1);
        plot_stl(['../data/stl/contour/configuration-',sensor_id(trial),'/tool-',sensor_id(trial),'-print4.STL'],rotation_tcp_k(:,:,j),position_tcp_k(j,:),0.001,part4_color,1);
    end
end

% Plot position trajectory
position_traj = squeeze(measured_pose_transf(1:3,4,:))';
plot3(position_traj(:,1),position_traj(:,2),position_traj(:,3),'b','LineWidth',2)

% Plot force trajectory
force_traj = measured_wrench_transf(1:3,:)';
for i=1:5:N
    mArrow3(position_tcp(i,:),position_tcp(i,:)+0.1*force_traj(i,:)/norm(force_traj(i,:)),'color',darkred,'stemWidth',0.0005*5,'facealpha',0.25);
end

% Plotting parameters
label_font_size = 25;
set(gcf,'units','normalized','outerposition',[0 0.04 0.7 0.74]);
set(gca,'FontSize',20)
hold on; axis equal; grid on; box on;

xlabel('$x\ [\mathrm{m}]$','Interpreter','LaTex','FontSize',label_font_size)
ylabel('$y\ [\mathrm{m}]$','Interpreter','LaTex','FontSize',label_font_size)
zlabel('$z\ [\mathrm{m}]$','Interpreter','LaTex','FontSize',label_font_size)

% xlim([-0.4 0.1])
% ylim([1.3 2.1])
% zlim([-0.6 -0.2])

view([103,47])


