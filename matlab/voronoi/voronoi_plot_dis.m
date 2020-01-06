function voronoi_plot_dis(x,y,Voronoi,Z,fieldInfo,targetInfo,Perception,E)
    global xlimit ylimit % plot
    global X Y weightScale
    global charge
    global Echarge
    
    subplot(1,2,1)
    
    hold off
    
%% unpack
Region = Voronoi.Region;

%% weight plot
    surf(X,Y,Z,'LineStyle','none')
    contourf(X,Y,Z)
    caxis([0 weightScale])
    colormap(jet)
    hold on
%% constraint plot
    plot(fieldInfo.plotX, fieldInfo.plotY, 'r', 'LineWidth', 3);
    hold on    

        for i=1:length(x)
            if Perception(i)
                plot(targetInfo(i).plotX, targetInfo(i).plotY, 'r', 'LineWidth', 3);
                hold on
            end
    end
    
%% charging station plot
plot(charge.plotx(:,:)', charge.ploty(:,:)', 'c', 'LineWidth', 5);
hold on

%% flag plot
%     plot(flagInfo.plotX, flagInfo.plotY, 'r:', 'LineWidth', 3);    
%% voronoi plot
    for i=1:length(x)
        % gather coordinate of points whose neaest agent is i
        voronoi_x = X(Region(:,:,i));
        voronoi_y = Y(Region(:,:,i));
        
        % convhull: calculate the points which is necessary to include them 
        %           by convex hull (and sort the points)
        %           faster than boundary
        point_index = convhull(voronoi_x,voronoi_y,'simplify',true);
        % voronoi boundary plot
        plot(voronoi_x(point_index),voronoi_y(point_index),'')
        hold on;
        
        % central plot
%         plot(x(i)+ux(i),y(i)+uy(i), 'm.', 'MarkerSize', 10)
%         hold on ;       
    end
%% agent position plot
    plot(x,y, 'w.', 'MarkerSize', 15)
    hold on
    

    
    
    xlim([xlimit(1)-0.1 xlimit(2)+0.1])
    ylim([ylimit(1)-0.1 ylimit(2)+0.1])
    axis equal
    
    subplot(1,2,2)
    bar(E)
    ylim([0 Echarge])
%     yline(Emin,'--','Minimum','LineWidth',3);
    title('Energy level')

    
    drawnow
end
