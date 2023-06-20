function plot_screw_invariants(progress,invars,datatype,parameterization,tab1)

% Pick the correct labels for the X- and Y-axis based on the given data type
if strcmp(parameterization,'dimless_arclength')
    label_x = '$\xi$ [-]'; % progress
elseif strcmp(parameterization,'time_based')
    label_x = '$t$ [s]'; % progress
end
if strcmp(datatype,'pose')
    % labels for pose data
    if strcmp(parameterization,'dimless_arclength')
        label_y = {'$a$ [rad/-]','$\omega_\kappa$ [rad/-]','$\omega_\tau$ [rad/-]','$b$ [m/-]','$v_b$ [m/-]','$v_t$ [m/-]'};
    elseif strcmp(parameterization,'time_based')
        label_y = {'$a$ [rad/s]','$\omega_\kappa$ [rad/s]','$\omega_\tau$ [rad/s]','$b$ [m/s]','$v_b$ [m/s]','$v_t$ [m/s]'};
    end
elseif strcmp(datatype,'wrench')
    % labels for wrench data
    if strcmp(parameterization,'dimless_arclength')
        label_y = {'$a$ [N]','$\omega_\kappa$ [rad/-]','$\omega_\tau$ [rad/-]','$b$ [Nm]','$v_b$ [m/-]','$v_t$ [m/-]'};
    elseif strcmp(parameterization,'time_based')
        label_y = {'$a$ [N]','$\omega_\kappa$ [rad/s]','$\omega_\tau$ [rad/s]','$b$ [Nm]','$v_b$ [m/s]','$v_t$ [m/s]'};
    end
else
    % generic labels
    label_y = {'$i_1$','$i_2$','$i_3$','$i_4$','$i_5$','$i_6$'};
end

axes('Parent',tab1);
% figure1 = figure('Name',['screw invariants of ',datatype],'Color',[1 1 1]);
set(gcf,'Units','normalized','OuterPosition',[0.4562    0.2250    0.6078    0.5648]);
for k=1:2
    for l=1:3
        idx = 3*(k-1)+l;
        subplots(idx) = subplot(2,3,idx,'Parent',tab1,'YGrid','on','FontSize',12);
        box(subplots,'on');
        hold(subplots,'all');
        plot(0,0);
                   
        plot(progress,invars(:,idx),'Parent',subplots(idx),'LineWidth',1,'LineStyle','-');
                
        if k==2
            xlabel(label_x,'Interpreter','LaTex','FontSize',18,'Rotation',0,'HorizontalAlignment', 'left');
        end
        title(label_y{3*(k-1)+l},'Interpreter','LaTex','FontSize',20,'Rotation',0,'HorizontalAlignment', 'center');
    end
end

% Link moving frame invariants top row and bottom row
linkaxes(subplots([2 3]),'y')
linkaxes(subplots([5 6]),'y')
linkaxes(subplots([2 3]),'off')
linkaxes(subplots([5 6]),'off')

%if ~isempty(titel)
%    suptitle(titel);
%end

end