function voronoi_plot_dis(x,y,cent,Region,Z,targetInfo,flagInfo)
    global xlimit ylimit % plot
    global X Y
    hold off
%% weight plot
    surf(X,Y,Z,'LineStyle','none')
    contourf(X,Y,Z)
    caxis([0 1])
    colormap(jet)
    hold on
%% constraint plot
    plot(targetInfo.plotX, targetInfo.plotY, 'r', 'LineWidth', 3);


%% flag plot
    plot(flagInfo.plotX, flagInfo.plotY, 'r:', 'LineWidth', 3);    
%% voronoi plot
    for i=1:length(x)
        % Regionに基づき，最も近いエージェントがiとなる点群の座標を�?める
        voronoi_x = X(Region(:,:,i));
        voronoi_y = Y(Region(:,:,i));
        
        % 点群全体をを�?�に�?�?ために�?要となる点のindexを求め?��ソートもする神関数
        % boundary()より圧倒的に早�?
        point_index = convhull(voronoi_x,voronoi_y,'simplify',true);
        % �?界線�?�ロ�?�?
        plot(voronoi_x(point_index),voronoi_y(point_index),'')
        hold on;
        
        % 重�?プロ�?�?
        plot(cent(i,1),cent(i,2), 'm.', 'MarkerSize', 10)
        hold on ;       
    end
%% agent position plot
    plot(x,y, 'w.', 'MarkerSize', 15)
    hold on
    

    
    
    xlim([xlimit(1)-0.1 xlimit(2)+0.1])
    ylim([ylimit(1)-0.1 ylimit(2)+0.1])
    axis equal
    drawnow
end

