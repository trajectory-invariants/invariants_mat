function plot_trajectory_coordinates(progress,data,datatype,parameterization)
% This function can plot many types of trajectory coordinates: position, orientation,
% pose, force, moment, wrench.

if strcmp(datatype,'pose')
    data = convert_posematrix_to_posecoordinates(data);
end

%% Settings plots
xlabel_fontsize = 18;
ylabel_fontsize = 20;
sgtitle_fontsize = 20;
dim = size(data,2);
nb_rows = dim/3;

%% Location of the figure
if dim==3 % orientation, position, force, moment
    set(gcf, 'Units', 'Normalized', 'OuterPosition', [0.2443    0.4546    0.5823    0.3296]); % (x0,y0,width,height)
elseif dim==6 % pose, wrench
    set(gcf, 'Units', 'Normalized', 'OuterPosition', [0.4562    0.2250    0.6078    0.5648]); % (x0,y0,width,height)
end

%% Plot subplots
[label_x, label_y] = axes_labels_trajectory(datatype, parameterization);
for idx = 1:dim
    subplot(nb_rows,3,idx)
    plot(progress,data(:,idx))

    set(gca,'YGrid','on')
    set(groot,'defaultAxesTickLabelInterpreter','latex');
    ax = gca; ax.XAxis.FontSize = 12; ax.YAxis.FontSize = 12;
    hold on

    xlabel(label_x,'Interpreter','LaTex','FontSize',xlabel_fontsize,'Rotation',0,'HorizontalAlignment','left');
    title(label_y{idx},'Interpreter','LaTex','FontSize',ylabel_fontsize,'Rotation',0,'HorizontalAlignment','center');
end

%% Place title of whole figure
if strcmp(datatype,'pose')
    title_text = 'measured (blue) vs reconstructed (red) pose';
elseif strcmp(datatype,'wrench')
    title_text = 'measured (blue) vs. reconstructed (red) wrench';
elseif strcmp(datatype,'position')
    title_text = 'measured (blue) vs reconstructed (red) position';
elseif strcmp(datatype,'orientation')
    title_text = 'measured (blue) vs reconstructed (red) orientation';
elseif strcmp(datatype,'force')
    title_text = 'measured (blue) vs reconstructed (red) force';
elseif strcmp(datatype,'moment')
    title_text = 'measured (blue) vs reconstructed (red) moment';
else % generic labels
    title_text = 'measured (blue) vs. reconstructed (red) trajectory coordinates';
end
sgtitle(title_text,'Interpreter','Latex','fontsize',sgtitle_fontsize);