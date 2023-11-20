function plot_figure10(T_isa, pose, trial, viewpoint, referencepoint, datatype, parameterization, application, view_fig, axis_font_size, label_font_size, step_size, wrenchtype)

figure('Name',['ISA frames of ',datatype],'Color',[1 1 1],'NumberTitle','off'); hold on; axis equal;
set(groot,'defaultAxesTickLabelInterpreter','latex');

% number identifying the location and orientation of the force sensor {lc} and motion tracker {tr} on the tool
sensor_id = configurations();

if strcmp(application,'contour')
    R_visual = rot_x(-135); % only for better visualization
else
    R_visual = eye(3) ;
end

nb_rigid_bodies = 3;

% Colors
part4_color = [120 120 120]/255;
part2_color = [80 80 80]/255;
bearings_color = [0 0 0]/255;
contour_color = [200 200 200]/255;

% ISA axis
arrow_length_isa = 0.2;
arrow_width_isa = 0.0005*6;
N_des = size(T_isa,3);

N = 101;
if strcmp(viewpoint,'world')
    
    % Fetch data to approximate the pose of the contour and tool
    if ~strcmp(trial,'ref') % not the reference case
        path_to_data_folder = './data/';
        [~,~,position_tcp,rotation_tcp,~,~,~,~,~,~,~,~,~,~] = contour_preprocess_data(N,viewpoint,parameterization,'tool_point',trial,trial,application,wrenchtype,path_to_data_folder);
        [rotation_tcp,position_tcp] = reverse_artificial_variations(rotation_tcp,position_tcp,trial);
    else
        [~,~,~,~,~,~,~,~,~,position_tcp,rotation_tcp,~,~,~] = contour_preprocess_data(N,viewpoint,parameterization,'tool_point',1,1,application,wrenchtype,path_to_data_folder);
    end
    
    % Choose some samples to plot by stl files
    k_1 = 1;
    for j = round(linspace(1,N_des,nb_rigid_bodies))
        rotation_tcp_k(:,:,k_1) = rotation_tcp(:,:,j);
        position_tcp_k(k_1,:)   = position_tcp(j,:);
        k_1                     = k_1+1;
    end
    
    if strcmp(application,'contour')
        % Estimate the pose of the contour
        load('data/stl/contour/contour_edge');
        contour_edge = contour_edge/1000; % convert mm to m
        average_contour_edge = mean(contour_edge,1);
        delta_contour_edge = contour_edge - average_contour_edge;
        average_position_tcp = mean(position_tcp,1);
        delta_position_tcp = position_tcp - average_position_tcp;
        % Calculate the spatial alignment
        S = delta_contour_edge'*delta_position_tcp;
        [U,~,V] = svd(S);
        ROTATION = V*U';
        if det(ROTATION) < 0
            ROTATION = V*[1,0,0;0,1,0;0,0,-1]*U';
        end
        TRANSLATION = (average_position_tcp' - ROTATION*average_contour_edge')';
        
        % Plot the contour
        plot_stl('data/stl/contour/contour.STL',ROTATION,TRANSLATION,0.001,contour_color,0.4)
        load('data/stl/contour/contour_edge_o')
        load('data/stl/contour/contour_edge_y')
        load('data/stl/contour/contour_edge_z')
        for j = 1 : N
            p_o_orig(j,:) = ROTATION*p_o_orig(j,:)'+TRANSLATION';
            p_y_orig(j,:) = ROTATION*p_y_orig(j,:)'+TRANSLATION';
            p_z_orig(j,:) = ROTATION*p_z_orig(j,:)'+TRANSLATION';
        end
        plot3(p_o_orig(:,1),p_o_orig(:,2),p_o_orig(:,3),'k','linewidth',1.5)
        plot3(p_y_orig(:,1),p_y_orig(:,2),p_y_orig(:,3),'k','linewidth',1.5)
        plot3(p_z_orig(:,1),p_z_orig(:,2),p_z_orig(:,3),'k','linewidth',1.5)
        plot3([p_o_orig(1,1),p_y_orig(1,1)],[p_o_orig(1,2),p_y_orig(1,2)],[p_o_orig(1,3),p_y_orig(1,3)],'k','linewidth',1.5)
        plot3([p_o_orig(1,1),p_z_orig(1,1)],[p_o_orig(1,2),p_z_orig(1,2)],[p_o_orig(1,3),p_z_orig(1,3)],'k','linewidth',1.5)
        plot3([p_o_orig(end,1),p_y_orig(end,1)],[p_o_orig(end,2),p_y_orig(end,2)],[p_o_orig(end,3),p_y_orig(end,3)],'k','linewidth',1.5)
        plot3([p_o_orig(end,1),p_z_orig(end,1)],[p_o_orig(end,2),p_z_orig(end,2)],[p_o_orig(end,3),p_z_orig(end,3)],'k','linewidth',1.5)
        plot3([p_y_orig(1,1),p_z_orig(1,1)],[p_y_orig(1,2),p_z_orig(1,2)],[p_y_orig(1,3),p_z_orig(1,3)],'k','linewidth',1.5)
        plot3([p_y_orig(end,1),p_z_orig(end,1)],[p_y_orig(end,2),p_z_orig(end,2)],[p_y_orig(end,3),p_z_orig(end,3)],'k','linewidth',1.5)

        % Plot stl files in correct position and orientation
        if ~strcmp(trial,'ref') % not the reference case
            for j = 1 : k_1-1
                plot_stl(['data/stl/contour/configuration-a/tool-a-bearings.STL'],rotation_tcp_k(:,:,j),position_tcp_k(j,:),0.001,bearings_color,1);
                plot_stl(['data/stl/contour/configuration-a/tool-a-print4.STL'],rotation_tcp_k(:,:,j),position_tcp_k(j,:),0.001,part4_color,1);
            end
        end
    elseif strcmp(application,'peg')
        ROTATION = position_tcp(end,:);
        TRANSLATION = eye(3);
        plot_stl(['data/stl/peg/hole.STL'],TRANSLATION,ROTATION,0.001,part2_color,0.2);
        % Plot stl files in correct position and orientation
        if ~strcmp(trial,'ref') % not the reference case
            for j = 1 : k_1-1
                plot_stl(['data/stl/peg/coupling.STL'],rotation_tcp_k(:,:,j),position_tcp_k(j,:),0.001,bearings_color,0.3);
                plot_stl(['data/stl/peg/peg.STL'],rotation_tcp_k(:,:,j),position_tcp_k(j,:),0.001,part4_color,0.3);
            end
        end
    end
    
    view(1,2)
    axis equal;


    darkred = [0.6350 0.0780 0.1840];
    red = [0.8500 0.3250 0.0980];
    green = [0.4660 0.6740 0.1880];
    blue = [0 0.4470 0.7410];
    grey = [0.8  0.8 0.8];
    
    T_ = zeros(4,4,N);
    for k = 1:N
        T_(:,:,k) = [T_isa(:,:,k); 0 0 0 1];
    end
    T_isa = T_;
    
    if (strcmp(referencepoint,'tracker') || strcmp(referencepoint,'tool_point'))
        if ~strcmp(trial,'ref')
            [~,~,T_isa] = reverse_artificial_variations(T_isa(1:3,1:3,:),squeeze(T_isa(1:3,4,:))',trial);
        end
    elseif strcmp(referencepoint,'force_sensor')
        for k = 1:N
            T_isa(:,:,k) = pose(:,:,k)*T_isa(:,:,k);
        end
        if ~strcmp(trial,'ref')
            [~,~,T_isa] = reverse_artificial_variations(T_isa(1:3,1:3,:),squeeze(T_isa(1:3,4,:))',trial);
        end
    end
    
    p_isa = squeeze(T_isa(1:3,4,:))';
    x_isa = squeeze(T_isa(1:3,1,:))';
    y_isa = squeeze(T_isa(1:3,2,:))';
    z_isa = squeeze(T_isa(1:3,3,:))';
    
    length_screw = 0.15;
    for i=1:step_size:N_des
        screw_start = p_isa(i,:) - x_isa(i,:)*length_screw*0.5;
        screw_end = p_isa(i,:) + x_isa(i,:)*length_screw;
        plot3([screw_start(1) screw_end(1)],[screw_start(2) screw_end(2)],[screw_start(3) screw_end(3)],'Color',grey,'LineWidth',0.5)
    end

   

    arrow_length_isa = 0.1;
    arrow_width_isa = 0.0015;
    for i=1:step_size:N_des
        mArrow3(p_isa(i,:),p_isa(i,:)+arrow_length_isa*x_isa(i,:)*1.2,'color',darkred,'stemWidth',arrow_width_isa*1.0);
        mArrow3(p_isa(i,:),p_isa(i,:)+arrow_length_isa*y_isa(i,:)*0.5,'color',green,'stemWidth',arrow_width_isa*0.50);
        mArrow3(p_isa(i,:),p_isa(i,:)+arrow_length_isa*z_isa(i,:)*0.5,'color',blue,'stemWidth',arrow_width_isa*0.50);
    end
    plot3(p_isa(:,1),p_isa(:,2),p_isa(:,3),'k','linewidth',2)
    
  
    
    set(gcf,'units','normalized','outerposition',[0 0.04 0.7 0.74]);
    set(gca,'FontSize',axis_font_size)
    axis equal; view(view_fig); grid on; box on;
    xlabel('$x\ [\mathrm{m}]$','Interpreter','LaTex','FontSize',label_font_size)
    ylabel('$y\ [\mathrm{m}]$','Interpreter','LaTex','FontSize',label_font_size)
    zlabel('$z\ [\mathrm{m}]$','Interpreter','LaTex','FontSize',label_font_size)
end

%% ISA of force in {tool}
if strcmp(viewpoint,'body')
    
    % Plot stl files in correct position and orientation
    if strcmp(application,'contour')
        plot_stl(['data/stl/contour/configuration-a/tool-a-bearings.STL'],R_visual,zeros(1,3),0.001,bearings_color,0.5);
        plot_stl(['data/stl/contour/configuration-a/tool-a-print4.STL'],R_visual,zeros(1,3),0.001,part4_color,0.5);
    elseif strcmp(application,'peg')
        plot_stl('data/stl/peg/coupling.STL',eye(3),zeros(1,3),0.001,bearings_color,0.3);
        plot_stl('data/stl/peg/peg.STL',eye(3),zeros(1,3),0.001,part4_color,0.3);
    end
    
    darkred = [0.6350 0.0780 0.1840];
    red = [0.8500 0.3250 0.0980];
    green = [0.4660 0.6740 0.1880];
    blue = [0 0.4470 0.7410];
    grey = [0.8  0.8 0.8];
    
    T_ = zeros(4,4,N);
    for k = 1:N
        T_(:,:,k) = [T_isa(:,:,k); 0 0 0 1];
    end
    T_isa = T_;
    
    if ~strcmp(trial,'ref') && strcmp(referencepoint,'tracker')
        if strcmp(application,'contour')
            [T_tcp_tr,~,~,~] = configuration_properties_contour(sensor_id(trial));
        elseif strcmp(application,'peg')
            [T_tcp_tr,~,~,~] = configuration_properties_peg();
        end
        for k = 1:N
            T_isa(:,:,k) = inverse_pose(T_tcp_tr)*T_isa(:,:,k);
        end
    end
    
    T_ = zeros(4,4,N);
    for k = 1:N
        T_(:,:,k) = [R_visual,[0;0;0]; 0 0 0 1]*T_isa(:,:,k);
    end
    T_isa = T_;
    
    p_isa = squeeze(T_isa(1:3,4,:,1))';
    x_isa = squeeze(T_isa(1:3,1,:,1))';
    y_isa = squeeze(T_isa(1:3,2,:,1))';
    z_isa = squeeze(T_isa(1:3,3,:,1))';
    
    arrow_length_isa = 0.1;
    arrow_width_isa = 0.0015;
    for i=1:step_size:N_des
        mArrow3(p_isa(i,:),p_isa(i,:)+arrow_length_isa*x_isa(i,:)*0.75,'color',darkred,'stemWidth',arrow_width_isa*0.5);
        mArrow3(p_isa(i,:),p_isa(i,:)+arrow_length_isa*y_isa(i,:)*0.5,'color',green,'stemWidth',arrow_width_isa*0.25);
        mArrow3(p_isa(i,:),p_isa(i,:)+arrow_length_isa*z_isa(i,:)*0.5,'color',blue,'stemWidth',arrow_width_isa*0.25);
    end
    plot3(p_isa(:,1),p_isa(:,2),p_isa(:,3))
    
    length_screw = 0.5*0.2;
    for i=1:step_size:N_des
        screw_start = p_isa(i,:) - 0.75*x_isa(i,:)*length_screw;
        screw_end = p_isa(i,:) + 0.75*x_isa(i,:)*length_screw;
        plot3([screw_start(1) screw_end(1)],[screw_start(2) screw_end(2)],[screw_start(3) screw_end(3)],'Color',grey,'LineWidth',0.75)
    end
    
    set(gcf,'units','normalized','outerposition',[0 0.04 0.7 0.74]);
    set(gca,'FontSize',axis_font_size)
    view(view_fig)
    hold on; axis equal; grid on; box on;
    xlabel('$x\ [\mathrm{m}]$','Interpreter','LaTex','FontSize',label_font_size)
    ylabel('$y\ [\mathrm{m}]$','Interpreter','LaTex','FontSize',label_font_size)
    zlabel('$z\ [\mathrm{m}]$','Interpreter','LaTex','FontSize',label_font_size)
    
end

