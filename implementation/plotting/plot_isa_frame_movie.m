function plot_isa_frame_movie(T_isa,arrow_length_isa,arrow_length_normal_binormal,arrow_width_isa,arrow_width_normal_binormal,steps_isa,darkred,green,blue)

N = size(T_isa,3);
p_isa = squeeze(T_isa(1:3,4,:))';
x_isa = squeeze(T_isa(1:3,1,:))';
y_isa = squeeze(T_isa(1:3,2,:))';
z_isa = squeeze(T_isa(1:3,3,:))';

for i=1:steps_isa:N
    mArrow3(p_isa(i,:),p_isa(i,:)+arrow_length_isa*x_isa(i,:),'color','r','stemWidth',arrow_width_isa);
    material dull
    hold on;
    mArrow3(p_isa(i,:),p_isa(i,:)+arrow_length_normal_binormal*y_isa(i,:),'color','g','stemWidth',arrow_width_normal_binormal);
    material dull
    hold on;
    mArrow3(p_isa(i,:),p_isa(i,:)+arrow_length_normal_binormal*z_isa(i,:),'color','b','stemWidth',arrow_width_normal_binormal);
    material dull
    hold on;
end

plot3(p_isa(:,1),p_isa(:,2),p_isa(:,3),'k','linewidth',2)
hold on;