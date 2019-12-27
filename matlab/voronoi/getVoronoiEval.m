function [J] = getVoronoiEval(x,y,Region,phi)
global X Y 
global pointDense b
J = zeros(length(x),1);
for i=1:length(x)
    voronoi_x = X(Region(:,:,i));
    voronoi_y = Y(Region(:,:,i));
    
    J(i) = -((voronoi_x-x(i)).^2+(voronoi_y-y(i)).^2+b)'*phi(Region(:,:,i))*pointDense...
            +b*(sum(sum(phi)))*pointDense/length(x);
end

