function plot_vector_invariants_contour(progress_ref,invars_ref,progress_all,invars_all,datatype)

% Pick the correct labels for the X- and Y-axis based on the given data type
label_x = '$\xi$ [-]'; % progress
if strcmp(datatype,'position')
    % labels for position data
    label_y = {'$c$ [m/-]','$\omega_\kappa$ [rad/-]','$\omega_\tau$ [rad/-]'};
elseif strcmp(datatype,'rotation')
    % labels for orientation data
    label_y = {'$c$ [rad/-]','$\omega_\kappa$ [rad/-]','$\omega_\tau$ [rad/-]'};
elseif strcmp(datatype,'force')
    % labels for force data
    label_y = {'$c$ [N]','$\omega_\kappa$ [rad/-]','$\omega_\tau$ [rad/-]'};
elseif strcmp(datatype,'moment')
    % labels for moment data
    label_y = {'$c$ [Nm]','$\omega_\kappa$ [rad/-]','$\omega_\tau$ [rad/-]'};
else
    % generic labels
    label_y = {'$i_1$','$i_2$','$i_3$'};
end

% Initialize figure and its position/size
figure1 = figure('Name',['vector invariants of ',datatype],'Color',[1 1 1]);
set(gcf,'Units','normalized','OuterPosition',[0.2443    0.4546    0.5823    0.3296]);
set(groot,'defaultAxesTickLabelInterpreter','latex');

for j=1:3 % three columns

    % initialize subplot
    subplots(j) = subplot(1,3,j,'Parent',figure1,'YGrid','on','FontSize',12);
    box(subplots,'on');
    hold(subplots,'all');
    plot(0,0);

    % plot variance bands for the chosen invariant of all trials
    std_invars = 2*std(squeeze(invars_all(:,j,:)),0,2);
    mean_invars = mean(squeeze(invars_all(:,j,:)),2);

    shadedplot(progress_ref,mean_invars'-std_invars',mean_invars'+std_invars',[0.9 0.9 0.9]); %second area is blue

    % plot the curve of the chosen invariant of all trials
    for i=1:size(invars_all,3)
        plot(progress_all(:,i),invars_all(:,j,i),'Parent',subplots(j),'LineWidth',1.5,'LineStyle','-');
    end

    % plot the curve of the chosen invariant of all trials
    plot(progress_ref,invars_ref(:,j),'Parent',subplots(j),'LineWidth',2.5,'Color',[0.0 0.0 1.0],'LineStyle',':');

    % dummy plots for the first invariant so that Y-axis is enlarged
    plot(0,-1,'Color',[0.0 0.0 1.0]);
    plot(0,+1,'Color',[0.0 0.0 1.0]);

    % show name of the horizontal axis
    xlabel(label_x,'Interpreter','LaTex','FontSize',18,'Rotation',0,'HorizontalAlignment', 'left');

    % show name of the invariant as the title of the subplot
    title(label_y{j},'Interpreter','LaTex','FontSize',20,'Rotation',0,'HorizontalAlignment', 'center');
    %ylabel(label_y{k},'Interpreter','LaTex','FontSize',20,'Rotation',90,'HorizontalAlignment', 'left');
end

% Link axes limits of the moving frame invariants
linkaxes(subplots([2 3]),'y')

% Turn off again to allow manual rescaling of individual subplots afterwards
linkaxes(subplots([2 3]),'off')

end