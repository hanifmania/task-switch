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
E = [4500 3500 4000 4000];
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
% Kd = 0;


m=0.6;
theta = linspace(0, 2*pi);
% Set charging station position
radius_charge = 0.15;
if AgentNum == 1
    charge.pos = [xlimit(1) + 2*radius_charge; ylimit(2) - 1];
    charge.pos = [mean(xlimit) ; mean(ylimit)];
else
    margin = 0.4;
    station_distance = (sum(abs(xlimit)) - 2*margin - 2*radius_charge)/(AgentNum-1);
    for i=1:AgentNum
        charge.pos(:,i) = [xlimit(1) + margin + radius_charge + station_distance*(i-1); ylimit(2) - 1];
    end
end
charge.plotx = [];
charge.ploty = [];
for i=1:AgentNum
    charge.plotx = [charge.plotx; radius_charge*sin(theta)+charge.pos(1,i)];
    charge.ploty = [charge.ploty; radius_charge*cos(theta)+charge.pos(2,i)];
end

