clear all
close all
clear java

addpath('voronoi')
addpath('optimization')
addpath('initialize')
addpath('grb')
addpath('functions')
% javaaddpath('jars/org.eclipse.paho.client.mqttv3-1.2.0.jar')
% javaaddpath('jars/Subscriber.jar')
% addpath(genpath(pwd))

global xlimit ylimit % plot
global R % neighbor ode
global X Y mesh_acc % mesh grid
global delta_increase delta_decrease perception_increase

%% Mode setting
real = 1;

% Plot in matlab or ROS.
matlab_plot = 1;
crazyflie = 0;
bebop = 1;


%% field settings
xlimit = [-1.7 2.0];
ylimit = [-1.2 1.4];


%%% draw mesh grid on field %%%%%%%
mesh_acc = [200 100];% mesh accuracy:[n m]でx方向n点y方向m点に�?割
xgrid = linspace(xlimit(1),xlimit(2),mesh_acc(1));
ygrid = linspace(ylimit(1),ylimit(2),mesh_acc(2));
[X,Y] = meshgrid(xgrid,ygrid);

num_grid_x = mesh_acc(1);
num_grid_y = mesh_acc(2);



%%% define importance weight %%%%%
mu = [0 0];
sigma = [0.25 0.3; 0.3 1];
xy = [X(:) Y(:)];
weight = mvnpdf(xy,mu,sigma);
weight = reshape(weight,num_grid_y,num_grid_x);
Z = weight;
Z = ones(num_grid_y, num_grid_x);% uniformly important...


%%% importance change parmeters
delta_increase = 0.001;
delta_decrease = 0.05;
perception_increase = 0.1;

%% agents settings
% エージェント台数
AgentNum = 1;

% エージェント�?�期値[-1 1]
x = -ones(1,AgentNum)+2*rand(1,AgentNum);
y = -ones(1,AgentNum)+2*rand(1,AgentNum);

R = 0.5;% 半径Rの認識�?囲


%% controller settings
k = 0.5;
ux = zeros(1,AgentNum);
uy = zeros(1,AgentNum);

%% simulation settings
T = 0.1;
simtime = 100;


%% CBF settings
pos = [sum(xlimit)/2; sum(ylimit)];   %���S
theta = [0];%��]�p(���ꂼ��̏�Q���ɑ΂��Đݒ�)
norm = [2]; %����(���ꂼ��̏�Q���ɑ΂��Đݒ�)
width = [0.1; 0.1];%x����
targetInfo = getPnomCBF(pos,theta,norm,width);

%  for flag CBF
flagInfo = getPnomCBF(pos,theta,norm,R-width);

%%
% sparseにしてもよさげ
Region = false(mesh_acc(2),mesh_acc(1),AgentNum);%x方向が列，y方向が行なのでmesh_accびindexが�??にな�?
cent = zeros(AgentNum,2);
Monitored = false(mesh_acc(2),mesh_acc(1),AgentNum);
Perception = zeros(1,AgentNum);

result = zeros(5,AgentNum);
u_opt = zeros(2,AgentNum);
for t=0:T:simtime
    Z = updateWeight(x,y,Z,Perception);
    [ux,uy,Region,cent]=voronoi_ode_dis(x,y,Z,k,true);
    voronoi_plot_dis(x,y,cent,Region,Z,targetInfo,flagInfo)
    
    for ii=1:AgentNum
        result(:,ii) = MIQP(x(ii),y(ii),[ux(ii);uy(ii)],targetInfo,flagInfo);
    end
%     if any(ux>5)|any(uy>5)
%         disp('big!!!!!!!!')
%     end
    result
%     CBF = f_con{1,1}{1,1}(x,y)
    u_opt = result(1:2,:);
    x = x+(u_opt(1,:))*T;
    y = y+(u_opt(2,:))*T;
    t;

end


