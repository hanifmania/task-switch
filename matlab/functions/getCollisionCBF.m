function [CBF] = getCollisionCBF(xi,yi,x_others,y_others,r)
clearance = 2*r;

    CBF.dhx = [2*(xi - x_others); 2*(yi - y_others)];
    CBF.hx = (xi - x_others).^2 + (yi - y_others).^2 - clearance.^2;


end

