function [d_lambda] = ode_lambda(xi,yi)


global X Y  % mesh grid
global l

    distance = (X(5,5)-xi)^2+(Y(5,5)-yi)^2;
    d_lambda = l*(distance-0.1^2); % consider a constraint dis-0.1^2<=0


end

