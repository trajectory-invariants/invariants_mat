function plot_trajectory_coordinates(progress,data,descriptor_type,title_text,parameterization)
% This function can plot many types of descriptors as listed in the following,
% descriptor_type:
% dim = 3
%     rotation
%     position
%     force
%     moment
% dim = 6:
%     pose
%     wrench

%% Initialization
xlabel_fontsize = 18;
ylabel_fontsize = 20;
sgtitle_fontsize = 20;

%% Location of the figure
if strcmp(descriptor_type,'rotation') || ...
        strcmp(descriptor_type,'position') || ...
        strcmp(descriptor_type,'force') || ...
        strcmp(descriptor_type,'moment')
    set(gcf, 'Units', 'Normalized', 'OuterPosition', [0.2443    0.4546    0.5823    0.3296]); % (x0,y0,width,height)
elseif strcmp(descriptor_type,'pose') || ...
        strcmp(descriptor_type,'wrench')
    set(gcf, 'Units', 'Normalized', 'OuterPosition', [0.4562    0.2250    0.6078    0.5648]); % (x0,y0,width,height)
end

%%
dim = size(data,2);

%% Plot
if dim == 3
    % rotation
    % position
    % force
    % moment
    if strcmp(descriptor_type,'rotation') || ...
            strcmp(descriptor_type,'position') || ...
            strcmp(descriptor_type,'force') || ...
            strcmp(descriptor_type,'moment')
        for idx = 1 : 3
            subplot(1,3,idx)
            plot(progress,data(:,idx))
            set(gca,'YGrid','on')
            set(groot,'defaultAxesTickLabelInterpreter','latex');
            ax = gca; ax.XAxis.FontSize = 12; ax.YAxis.FontSize = 12;
            hold on
        end
    end
elseif dim == 6
    % pose
    % wrench
    if strcmp(descriptor_type,'pose') || ...
            strcmp(descriptor_type,'wrench')
        for idx = 1 : 6
            subplot(2,3,idx)
            plot(progress,data(:,idx))
            set(gca,'YGrid','on')
            ax = gca; ax.XAxis.FontSize = 12; ax.YAxis.FontSize = 12;
            hold on
        end
    end
end

sgtitle(title_text,'Interpreter','Latex','fontsize',sgtitle_fontsize)

%% Labels
% rotation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if strcmp(descriptor_type,'rotation')
    if  strcmp(parameterization,'time_based')
        % roll
        subplot(1,3,1)
        xlabel('$t\ [\mathrm{s}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$roll\ [\mathrm{rad}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
        % pitch
        subplot(1,3,2)
        xlabel('$t\ [\mathrm{s}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$pitch\ [\mathrm{rad}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
        % yaw
        subplot(1,3,3)
        xlabel('$t\ [\mathrm{s}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$yaw\ [\mathrm{rad}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
    elseif strcmp(parameterization,'dimless_arclength')
        % roll
        subplot(1,3,1)
        xlabel('$\xi$ [-]','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$roll\ [\mathrm{rad}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
        % pitch
        subplot(1,3,2)
        xlabel('$\xi$ [-]','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$pitch\ [\mathrm{rad}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
        % yaw
        subplot(1,3,3)
        xlabel('$\xi$ [-]','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$yaw\ [\mathrm{rad}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
    end
    % position %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif strcmp(descriptor_type,'position')
    if  strcmp(parameterization,'time_based')
        % px
        subplot(1,3,1)
        xlabel('$t\ [\mathrm{s}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$p_x\ [\mathrm{m}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
        % py
        subplot(1,3,2)
        xlabel('$t\ [\mathrm{s}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$p_y\ [\mathrm{m}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
        % pz
        subplot(1,3,3)
        xlabel('$t\ [\mathrm{s}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$p_z\ [\mathrm{m}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
    elseif strcmp(parameterization,'dimless_arclength')
        % px
        subplot(1,3,1)
        xlabel('$\xi$ [-]','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$p_x\ [\mathrm{m}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
        % py
        subplot(1,3,2)
        xlabel('$\xi$ [-]','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$p_y\ [\mathrm{m}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
        % pz
        subplot(1,3,3)
        xlabel('$\xi$ [-]','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$p_z\ [\mathrm{m}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
    end
    % force %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif strcmp(descriptor_type,'force')
    if  strcmp(parameterization,'time_based')
        % fx
        subplot(1,3,1)
        xlabel('$t\ [\mathrm{s}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$f_x\ [\mathrm{N}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
        % fy
        subplot(1,3,2)
        xlabel('$t\ [\mathrm{s}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$f_y\ [\mathrm{N}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
        % fz
        subplot(1,3,3)
        xlabel('$t\ [\mathrm{s}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$f_z\ [\mathrm{N}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
    elseif strcmp(parameterization,'dimless_arclength')
        % fx
        subplot(1,3,1)
        xlabel('$\xi$ [-]','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$f_x\ [\mathrm{N}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
        % fy
        subplot(1,3,2)
        xlabel('$\xi$ [-]','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$f_y\ [\mathrm{N}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
        % fz
        subplot(1,3,3)
        xlabel('$\xi$ [-]','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$f_z\ [\mathrm{N}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
    end
    % moment %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif strcmp(descriptor_type,'moment')
    if  strcmp(parameterization,'time_based')
        % mx
        subplot(1,3,1)
        xlabel('$t\ [\mathrm{s}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$m_x\ [\mathrm{N.m}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
        % my
        subplot(1,3,2)
        xlabel('$t\ [\mathrm{s}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$m_y\ [\mathrm{N.m}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
        % mz
        subplot(1,3,3)
        xlabel('$t\ [\mathrm{s}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$m_z\ [\mathrm{N.m}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
    elseif strcmp(parameterization,'dimless_arclength')
        % mx
        subplot(1,3,1)
        xlabel('$\xi$ [-]','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$m_x\ [\mathrm{N.m}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
        % my
        subplot(1,3,2)
        xlabel('$\xi$ [-]','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$m_y\ [\mathrm{N.m}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
        % mz
        subplot(1,3,3)
        xlabel('$\xi$ [-]','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$m_z\ [\mathrm{N.m}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
    end
    % pose %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif strcmp(descriptor_type,'pose')
    if  strcmp(parameterization,'time_based')
        % roll
        subplot(2,3,1)
        title('$roll\ [\mathrm{rad}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        % pitch
        subplot(2,3,2)
        title('$pitch\ [\mathrm{rad}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        % yaw
        subplot(2,3,3)
        title('$yaw\ [\mathrm{rad}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        % px
        subplot(2,3,4)
        xlabel('$t\ [\mathrm{s}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$p_x\ [\mathrm{m}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        % py
        subplot(2,3,5)
        xlabel('$t\ [\mathrm{s}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$p_y\ [\mathrm{m}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        % pz
        subplot(2,3,6)
        xlabel('$t\ [\mathrm{s}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$p_z\ [\mathrm{m}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
    elseif  strcmp(parameterization,'dimless_arclength')
        % roll
        subplot(2,3,1)
        title('$roll\ [\mathrm{rad}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        % pitch
        subplot(2,3,2)
        title('$pitch\ [\mathrm{rad}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        % yaw
        subplot(2,3,3)
        title('$yaw\ [\mathrm{rad}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        % px
        subplot(2,3,4)
        xlabel('$\xi\ [\mathrm{-}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$p_x\ [\mathrm{m}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
        % py
        subplot(2,3,5)
        xlabel('$\xi\ [\mathrm{-}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$p_y\ [\mathrm{m}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
        % pz
        subplot(2,3,6)
        xlabel('$\xi\ [\mathrm{-}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$p_z\ [\mathrm{m}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
    end
    % wrench %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif strcmp(descriptor_type,'wrench')
    if  strcmp(parameterization,'time_based')
        % fx
        subplot(2,3,1)
        title('$f_x\ [\mathrm{N}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        % fy
        subplot(2,3,2)
        title('$f_y\ [\mathrm{N}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        % fz
        subplot(2,3,3)
        title('$f_z\ [\mathrm{N}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        % mx
        subplot(2,3,4)
        xlabel('$t\ [\mathrm{s}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$m_x\ [\mathrm{N.m}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
        % mx
        subplot(2,3,5)
        xlabel('$t\ [\mathrm{s}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$m_y\ [\mathrm{N.m}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
        % mz
        subplot(2,3,6)
        xlabel('$t\ [\mathrm{s}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$m_z\ [\mathrm{N.m}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
    elseif strcmp(parameterization,'dimless_arclength')
        % fx
        subplot(2,3,1)
        title('$f_x\ [\mathrm{N}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        % fy
        subplot(2,3,2)
        title('$f_y\ [\mathrm{N}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        % fz
        subplot(2,3,3)
        title('$f_z\ [\mathrm{N}]$','Interpreter','Latex','fontsize',xlabel_fontsize)
        % mx
        subplot(2,3,4)
        xlabel('$\xi$ [-]','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$m_x\ [\mathrm{N.m}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
        % my
        subplot(2,3,5)
        xlabel('$\xi$ [-]','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$m_y\ [\mathrm{N.m}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
        % mz
        subplot(2,3,6)
        xlabel('$\xi$ [-]','Interpreter','Latex','fontsize',xlabel_fontsize)
        title('$m_z\ [\mathrm{N.m}]$','Interpreter','Latex','fontsize',ylabel_fontsize)
    end
end
end

