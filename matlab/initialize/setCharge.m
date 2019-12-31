%%% Use CBF that agent moves to charging station with fixed velocity or not
CBF_fixed = 1;
%%% Set the gain used in CBF. It affects moving speed to charging station.
if CBF_fixed
%     In this case, k_charge = max velocity.(saturate.)
    k_charge = 0.2;
else
    k_charge = 20;
end
% initial Energy level
E = [4500 3500];
Echarge = 4500;
Emin = 1500;
% Eplot = zeros(AgentNum,num_step);

charge_flag = zeros(1,AgentNum);
land_flags = zeros(1, AgentNum);
% Parameters for Improved Energy Model
charge_I = 0.5; % I-Ic > 0
discharge_I = 0.5; % I-Ic < 0
charge_k = 1;
lam = 1;

Kd = 50;


m=0.6;
theta = linspace(0, 2*pi);
% Set charging station position
radius_charge = 0.15;
station_distance = (sum(abs(xlimit)) - 2*radius_charge)/(AgentNum-1);
pos_charge = zeros(2,AgentNum);
for i=1:AgentNum
    pos_charge(:,i) = [xlimit(1) + radius_charge + station_distance*(i-1); ylimit(2) - 0.6];
end
plot_charge_x = [];
plot_charge_y = [];
for i=1:AgentNum
    plot_charge_x = [plot_charge_x; radius_charge*sin(theta)+pos_charge(1,i)];
    plot_charge_y = [plot_charge_y; radius_charge*cos(theta)+pos_charge(2,i)];
end

