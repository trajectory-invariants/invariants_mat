function plot_FS_frames(invariant_frame, position_trajectory)

figure('Name',['FS frames'],'Color',[1 1 1]); hold on; axis equal;

% plotting parameters
arrow_length_isa = 0.05; 
arrow_width_isa = 0.0005*6; 
axis_width_isa = 2;
axis_length_isa = 0.1; view_fig = [-50,10]; %step_plot = 2;

N_des = size(invariant_frame,3);

darkred = [0.6350 0.0780 0.1840];
red = [0.8500 0.3250 0.0980];
green = [0.4660 0.6740 0.1880];
blue = [0 0.4470 0.7410];
grey = [0.8  0.8 0.8];

% extract axis frames
e_x = squeeze(invariant_frame(1:3,1,:))';
e_y = squeeze(invariant_frame(1:3,2,:))';
e_z = squeeze(invariant_frame(1:3,3,:))';

arrow_width_isa = 0.0015;
for i=1:5:N_des
    mArrow3(position_trajectory(i,:),position_trajectory(i,:)+arrow_length_isa*e_x(i,:),'color',darkred,'stemWidth',arrow_width_isa);
    mArrow3(position_trajectory(i,:),position_trajectory(i,:)+arrow_length_isa*e_y(i,:),'color',green,'stemWidth',arrow_width_isa*0.50);
    mArrow3(position_trajectory(i,:),position_trajectory(i,:)+arrow_length_isa*e_z(i,:),'color',blue,'stemWidth',arrow_width_isa*0.50);
end

plot3(position_trajectory(:,1),position_trajectory(:,2),position_trajectory(:,3),'r')

axis_font_size = 12;
label_font_size = 18;
set(gcf,'units','normalized','outerposition',[0 0.04 0.7 0.74]);
set(gca,'FontSize',axis_font_size)
hold on; axis equal; view(view_fig); grid on; box on;
xlabel('$x\ [\mathrm{m}]$','Interpreter','LaTex','FontSize',label_font_size)
ylabel('$y\ [\mathrm{m}]$','Interpreter','LaTex','FontSize',label_font_size)
zlabel('$z\ [\mathrm{m}]$','Interpreter','LaTex','FontSize',label_font_size)

end
