function plot_vector_invariants(progress,invars,datatype,parameterization,tab1)

% Pick the correct labels for the X- and Y-axis based on the given data type
if strcmp(parameterization,'dimless_arclength')
    label_x = '$\xi$ [-]'; % progress
elseif strcmp(parameterization,'time_based')
    label_x = '$t$ [s]'; % progress
end

if strcmp(datatype,'position')
    % labels for position data
    if strcmp(parameterization,'dimless_arclength')
        label_y = {'$c$ [m/-]','$\omega_\kappa$ [rad/-]','$\omega_\tau$ [rad/-]'};
    elseif strcmp(parameterization,'time_based')
        label_y = {'$c$ [m/s]','$\omega_\kappa$ [rad/s]','$\omega_\tau$ [rad/s]'};
    end
elseif strcmp(datatype,'orientation')
    % labels for orientation data
    if strcmp(parameterization,'dimless_arclength')
        label_y = {'$c$ [rad/-]','$\omega_\kappa$ [rad/-]','$\omega_\tau$ [rad/-]'};
    elseif strcmp(parameterization,'time_based')
        label_y = {'$c$ [rad/s]','$\omega_\kappa$ [rad/s]','$\omega_\tau$ [rad/s]'};
    end
elseif strcmp(datatype,'force')
    % labels for force data
    if strcmp(parameterization,'dimless_arclength')
        label_y = {'$c$ [N]','$\omega_\kappa$ [rad/-]','$\omega_\tau$ [rad/-]'};
    elseif strcmp(parameterization,'time_based')
        label_y = {'$c$ [N]','$\omega_\kappa$ [rad/s]','$\omega_\tau$ [rad/s]'};
    end
elseif strcmp(datatype,'moment')
    % labels for moment data
    if strcmp(parameterization,'dimless_arclength')
        label_y = {'$c$ [Nm]','$\omega_\kappa$ [rad/-]','$\omega_\tau$ [rad/-]'};
    elseif strcmp(parameterization,'time_based')
        label_y = {'$c$ [Nm]','$\omega_\kappa$ [rad/s]','$\omega_\tau$ [rad/s]'};
    end
else
    % generic labels
    label_y = {'$i_1$','$i_2$','$i_3$'};
end

axes('Parent',tab1);
% figure1 = figure('Name',['vector invariants of ',datatype],'Color',[1 1 1]);
set(gcf,'Units','normalized','OuterPosition',[0.2443    0.4546    0.5823    0.3296]);
for k=1:3
    subplots(k) = subplot(1,3,k,'Parent',tab1,'YGrid','on','FontSize',12);
    box(subplots,'on');
    hold(subplots,'all');
    plot(0,0);
    %ylim([-1 1])

    for i=1:size(invars,3)
        plot(progress,invars(:,k),'Parent',subplots(k),'LineWidth',1,'LineStyle','-');
    end

    xlabel(label_x,'Interpreter','LaTex','FontSize',18,'Rotation',0,'HorizontalAlignment', 'left');
    %ylabel(label_y{k},'Interpreter','LaTex','FontSize',20,'Rotation',90,'HorizontalAlignment', 'left');
    title(label_y{k},'Interpreter','LaTex','FontSize',20,'Rotation',0,'HorizontalAlignment', 'center');
end

% Link Y-axes of moving frame invariants top row and bottom row
linkaxes(subplots([2 3]),'y')
%linkaxes(subplots([5 6]),'y')
linkaxes(subplots([2 3]),'off')

%if ~isempty(titel)
%    suptitle(titel);
%end

end