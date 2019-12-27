function [val] = Evalfunc(x,y,Region,Z)
    
global X Y mesh_acc
persistent AgentNum
    if isempty(AgentNum)
        AgentNum = size(Region,3);
        weighted = zeros(1,AgentNum);
    end
for i=1:AgentNum
    voronoi_x = X(Region(:,:,i));
    voronoi_y = Y(Region(:,:,i));
    performance = (voronoi_x-x(i)).^2+(voronoi_y-y(i)).^2;
    weighted(i) = performance'*Z(Region(:,:,i));
end
val = sum(weighted,2);
end

