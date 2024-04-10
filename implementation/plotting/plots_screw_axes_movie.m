function plots_screw_axes_movie(T_isa,length_screw_back,length_screw_front,grey,width_axis,steps_isa)

N = size(T_isa,3);
p_isa = squeeze(T_isa(1:3,4,:))';
x_isa = squeeze(T_isa(1:3,1,:))';
%y_isa = squeeze(T_isa(1:3,2,:))';
%z_isa = squeeze(T_isa(1:3,3,:))';

for i=1:steps_isa:N
    screw_start = p_isa(i,:) - x_isa(i,:)*length_screw_back;
    screw_end = p_isa(i,:) + x_isa(i,:)*length_screw_front;
    plot3([screw_start(1) screw_end(1)],[screw_start(2) screw_end(2)],[screw_start(3) screw_end(3)],'Color','r','LineWidth',width_axis)
end

