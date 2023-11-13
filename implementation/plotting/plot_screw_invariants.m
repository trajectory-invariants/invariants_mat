function plot_screw_invariants(progress,invars,datatype,parameterization,tab1)

[label_x, label_y] = determine_axes_labels(datatype, parameterization);

axes('Parent',tab1);
% figure1 = figure('Name',['screw invariants of ',datatype],'Color',[1 1 1]);
set(gcf,'Units','normalized','OuterPosition',[0.4562    0.2250    0.6078    0.5648]);

subplots = zeros(6,1);
for k=1:2
    for l=1:3
        idx = 3*(k-1)+l; % index of plot and of invariant
        
        % define subplot
        subplots(idx) = subplot(2,3,idx,'Parent',tab1,'YGrid','on','FontSize',12);
        box(subplots(idx),'on');
        hold(subplots(idx),'all');

        plot(0,0); % plot (0,0) to ensure that zero line is always included
        plot(progress,invars(:,idx),'Parent',subplots(idx),'LineWidth',1,'LineStyle','-'); % plot invariant

        if k==2
            % only display label of x-axis for bottom row of plots
            xlabel(label_x,'Interpreter','LaTex','FontSize',18,'Rotation',0,'HorizontalAlignment', 'left');
        end
        % y-label is put on top of the subfigure
        title(label_y{3*(k-1)+l},'Interpreter','LaTex','FontSize',20,'Rotation',0,'HorizontalAlignment', 'center');
    end
end

% Link axes limits of the moving frame invariants top row and bottom row
linkaxes(subplots([2 3]),'y')
linkaxes(subplots([5 6]),'y')

% Turn off again to allow manual rescaling of individual subplots afterwards
linkaxes(subplots([2 3]),'off') 
linkaxes(subplots([5 6]),'off') 

%if ~isempty(titel)
%    suptitle(titel);
%end


end