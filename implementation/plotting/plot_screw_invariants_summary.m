function plot_screw_invariants_summary(bool_reference_invariants,progress_ref,invars_ref,progress_all,invars_all,datatype,parameterization)
% Plot the invariant descriptors of all trials for the
% chosen trajectory type together with the reference invariants.
% In addition, show the variance bands around the invariant values.
%
% Input: bool_reference_invariants (bool) =
%        progress_ref              (Nx1)  =
%        invars_ref                =
%        progress                  =
%        invars_all                =
%        datatype                  =
%        parameterization          =

% Pick the correct labels for the X- and Y-axis based on the given data type
[label_x, label_y] = axes_labels_invariants(datatype, parameterization);

% Initialize figure and its position/size
figure1 = figure('Name',['screw invariants of ',datatype],'Color',[1 1 1],'NumberTitle','off');
set(gcf,'Units','normalized','OuterPosition',[0.3068    0.1361    0.7572    0.6537]);
set(groot,'defaultAxesTickLabelInterpreter','latex');
subplots = zeros(6,1);

for i=1:2 % top row, bottom row
    for j=1:3 % three columns
        idx = 3*(i-1)+j; % index of one of the six invariants to be plotted

        % initialize subplot
        subplots(idx) = subplot(2,3,idx,'Parent',figure1,'YGrid','on','FontSize',22);
        box(subplots(idx),'on');
        hold(subplots(idx),'all');
        plot(0,0);
        %set(gca,'FontSize',15)

        % plot variance bands for the chosen invariant of all trials
        std_invars = 2*std(squeeze(invars_all(:,idx,:)),0,2);
        mean_invars = mean(squeeze(invars_all(:,idx,:)),2);
        shadedplot(progress_ref,mean_invars'-std_invars',mean_invars'+std_invars',[0.9 0.9 0.9]); %second area is blue


        % plot the curve of the chosen invariant of all trials
        for n=1:size(invars_all,3)
            plot(progress_all(:,n),invars_all(:,idx,n),'Parent',subplots(idx),'LineWidth',1,'LineStyle','-'); %'Color',[1.0 0.0 0.0],'LineStyle',linestyles{i});
        end

        % plot the curve of the reference invariant (limit case)
        if bool_reference_invariants
            plot(progress_ref,invars_ref(:,idx),'Parent',subplots(idx),'LineWidth',2.5,'Color',[0.0 0.0 1.0],'LineStyle',':');
        end

        % dummy plots so that -1 and +1 are included in the Y-axis
        plot(0,-1,'Color',[0.0 0.0 1.0]);
        plot(0,1,'Color',[0.0 0.0 1.0]);

        % for the bottom row, show the label of the X-axis
        if i==2
            xlabel(label_x,'Interpreter','LaTex','FontSize',22,'Rotation',0,'HorizontalAlignment', 'left');
        end

        % show name of the invariant as the title of the subplot
        title(label_y{3*(i-1)+j},'Interpreter','LaTex','FontSize',26,'Rotation',0,'HorizontalAlignment', 'center');
    end
end

% Link axes limits of the moving frame invariants top row and bottom row
linkaxes(subplots([2 3]),'y')
linkaxes(subplots([5 6]),'y')

% Turn off again to allow manual rescaling of individual subplots afterwards
linkaxes(subplots([2 3]),'off')
linkaxes(subplots([5 6]),'off')

if strcmp(datatype,'wrench') && strcmp(viewpoint,'body') && strcmp(wrenchtype,'real')
    subplot(2,3,5); ylim([-0.2,0.2]); subplot(2,3,6); ylim([-0.1,0.1]);
    exportgraphics(gcf,['figures/screw_invariants_',datatype,'_',viewpoint,'_',referencepoint,'.pdf'],'ContentType','vector');
end
if strcmp(datatype,'wrench') && strcmp(wrenchtype,'synthetic')
    subplot(2,3,1); ylim([-1,35]); subplot(2,3,2); ylim([-5,5]); %subplot(2,3,3); ylim([-5,5])
    exportgraphics(gcf,['figures/screw_invariants_',datatype,'_',viewpoint,'_',referencepoint,'_',wrenchtype,'.pdf'],'ContentType','vector');
end

end