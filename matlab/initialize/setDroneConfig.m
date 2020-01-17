%% Initial setting



% Drone number list for experiment. Ex. [1, 3, 6]
drone_list = [2 3];
AgentNum = size(drone_list,2);

% sensor R: vision sensor radius
if bebop
    R = 0.5;
elseif crazyflie
    R = 0.2;
end

% Drone r for collision avoidance
% CBF force them to keep 2*r distance
r = 0.55;


% Set desired height of drone.(only for bebop2)
z_ref = 1.2;
% Set height thresh hold for taking off (only for bebop2)
z_thresh = 0.7;
kz = 0.4;


% set initial position
if ~real
    x = mean(xlimit) + (xlimit(2)-xlimit(1)).*(gallery('uniformdata',[1 AgentNum], 0)-0.6);
    y = mean(ylimit) + (ylimit(2)-ylimit(1)).*(gallery('uniformdata',[1 AgentNum], 1)-0.6);
    z = ones(1,AgentNum);

%     x = [-1.2, -1.2, -1.2];
%     y = [0, drone_r*2, -drone_r*2];
%     z = [1, 1, 1];

%     x = [-1.2, -1.2];
%     y = [drone_r*2, -drone_r*2];
%     z = [1, 1];

end
