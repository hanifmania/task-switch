function [CBF] = getPnomCBF(pos,theta,norm,width)
global xlimit ylimit % plot


CBF.pos = pos;
CBF.theta = theta;
CBF.norm = norm;
CBF.width = width;


CBF.hx = @(x,y) -(((x-pos(1))*cos(theta)+...
    (y-pos(2))*sin(theta))/width(1)).^norm...
    -((-(x-pos(1))*sin(theta)...
    +(y-pos(2))*cos(theta))/width(2)).^norm+1;

temp_plot = fimplicit(CBF.hx,'XRange',xlimit*1.1,'YRange',ylimit*1.1,'visible','off');
CBF.plotX = temp_plot.XData;
CBF.plotY = temp_plot.YData;
end

