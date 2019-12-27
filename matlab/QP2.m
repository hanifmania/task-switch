function [u_opt] = QP2(x, y, u_nom, x_others, y_others, E, pos_charge)

global OBJECT_NUM
global hx_o hx_f hx_c
global drone_r
global theta_obj norm_obj xwidth_obj ywidth_obj f_obj pos_obj
global theta_fie norm_fie xwidth_fie ywidth_fie f_fie pos_fie
global radius_charge Emin Kc k_charge CBF_fixed 


pos_robot = [x,y];

persistent n
if isempty(n)
    n = 0;
end
n = n+1;

% Charging CBF

if CBF_fixed
    h_charge = E - Emin - (Kc/k_charge) * (norm(pos_robot-pos_charge') - radius_charge);
else
    h_charge = E - Emin - (Kc/k_charge) * log(norm(pos_robot-pos_charge')/radius_charge);
end

if CBF_fixed
    A_charge = (Kc/k_charge) * ( (pos_robot-pos_charge')/norm(pos_robot-pos_charge') );
else
    A_charge = (Kc/k_charge) * ( (pos_robot-pos_charge')/norm(pos_robot-pos_charge')^2 );
end
b_charge = h_charge - Kc;

CBFscale = 100;
A_charge = A_charge./CBFscale;
b_charge = b_charge./CBFscale;




%%%field
dh_f = zeros(1,2);
hx_f = zeros(1,1);
coef = zeros(1,2);
%%% \partial x,yの計算にあたり，共通の係数だけ先に計算
coef(1) = -norm_fie*(((pos_robot(1)-pos_fie(1))*cos(theta_fie)...
    +(pos_robot(2)-pos_fie(2))*sin(theta_fie))/xwidth_fie).^(norm_fie-1);
coef(2) = -norm_fie*((-(pos_robot(1)-pos_fie(1))*sin(theta_fie)...
    +(pos_robot(2)-pos_fie(2))*cos(theta_fie))/ywidth_fie).^(norm_fie-1);

dh_f(:) = -[coef(1)*cos(theta_fie)/xwidth_fie+coef(2)*sin(theta_fie)/ywidth_fie,...
    coef(1)*sin(theta_fie)/xwidth_fie+coef(2)*cos(theta_fie)/ywidth_fie];
hx_f = f_fie(pos_robot(1),pos_robot(2));




dh_o = zeros(OBJECT_NUM,2);
hx_o = zeros(OBJECT_NUM,1);
coef = zeros(1,2);
for i = 1:OBJECT_NUM
    %%% \partial x,yの計算にあたり，共通の係数だけ先に計算
    coef(1) = -norm_obj(i)*(((pos_robot(1)-pos_obj(1,i))*cos(theta_obj(i))...
        +(pos_robot(2)-pos_obj(2,i))*sin(theta_obj(i)))/xwidth_obj(i)).^(norm_obj(i)-1);
    coef(2) = -norm_obj(i)*((-(pos_robot(1)-pos_obj(1,i))*sin(theta_obj(i))...
        +(pos_robot(2)-pos_obj(2,i))*cos(theta_obj(i)))/ywidth_obj(i)).^(norm_obj(i)-1);
    
    dh_o(i,:) = [coef(1)*cos(theta_obj(i))/xwidth_obj(i)+coef(2)*sin(theta_obj(i))/ywidth_obj(i),...
        coef(1)*sin(theta_obj(i))/xwidth_obj(i)+coef(2)*cos(theta_obj(i))/ywidth_obj(i)];
    hx_o(i) = -f_obj{1,i}{1,1}(pos_robot(1),pos_robot(2));
%    temp(i,:) = [(pos_robot(1) - pos_obj(1,i)), (pos_robot(2) - pos_obj(2,i))].*2;
end

dh_c = zeros(length(x_others),2);
hx_c = zeros(length(x_others),1);
% to keep 2*r distance
clearance = 2*drone_r;
for i = 1:length(x_others)
    dh_c(i,:) = [-2*(pos_robot(1) - x_others(i)), -2*(pos_robot(2) - y_others(i))];
    hx_c(i) = (pos_robot(1) - x_others(i)).^2 + (pos_robot(2) - y_others(i)).^2 - clearance.^2;
end

H = eye(2);
f = -u_nom;

gamma = 10;
A_avoid = [dh_o;dh_c;dh_f];
b_avoid = gamma * [10*hx_o.^3;10*hx_c.^3;10*hx_f.^3];


A = [A_charge;A_avoid];
b = [b_charge;b_avoid];

options = optimoptions('quadprog', 'Display', 'off');

%u_opt = quadprog(H,f,A,b);
try
    [u_opt] = quadprog(H,f,A,b, [], [], [], [], [], options);
catch
    warning('QuadProg ERROR');
    u_opt = [0;0];
end


end