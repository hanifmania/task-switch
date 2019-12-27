
% Set control gain
if real
    k = 0.5;
else
    k = 0.6;
end
% Set control gain for height (only for bebop2)
Kz = 0.3;

% PIDcontroller
% mode1 is for xy. mode0 is for z
Kp1 = 1.2;
Ki1 = 0.14;
Kd1 = 0.10;
b1 = 1;
c1 = 1;


% Kp0 = 1.9;
% Ki0 = 0.37;
% Kd0 = 0.10;
% b0 = 1;
% c0 = 1;
Kp0 = 1.2;
Ki0 = 0.14;
Kd0 = 0.10;
b0 = 1;
c0 = 1;

% ux = zeros(1,AgentNum);
% uy = zeros(1,AgentNum);
u_nom = zeros(2,AgentNum);
u_opt = zeros(2,AgentNum);
u_z = zeros(1,AgentNum);
optresult = zeros(5,AgentNum);