function [CBF] = getCollisionCBF(xi,yi,x_others,y_others)
global R
clearance = 2*R;

    CBF.dhx = [2*(xi - x_others); 2*(yi - y_others)];
    CBF.hx = (xi - x_others).^2 + (yi - y_others).^2 - clearance.^2;


end

