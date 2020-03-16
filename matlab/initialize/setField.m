%% field settings
xlimit = [-1.2 1.2];
ylimit = [-1.8 1.5];


%%% draw mesh grid on field %%%%%%%
mesh_acc = [200 100];% mesh accuracy:[n m]でx方向n点y方向m点に�?割
mesh_acc = [100 80];% mesh accuracy:[n m]でx方向n点y方向m点に�?割
xgrid = linspace(xlimit(1),xlimit(2),mesh_acc(1));
ygrid = linspace(ylimit(1),ylimit(2),mesh_acc(2));
[X,Y] = meshgrid(xgrid,ygrid);

num_grid_x = mesh_acc(1);
num_grid_y = mesh_acc(2);

pointDense = (xlimit(2)-xlimit(1))*(ylimit(2)-ylimit(1))/(mesh_acc(1)*mesh_acc(2));

%%% define importance weight %%%%%
mu = [0 0];
sigma = [0.25 0.3; 0.3 1];
xy = [X(:) Y(:)];

weightScale = 1;

weight = mvnpdf(xy,mu,sigma);
weight = reshape(weight,num_grid_y,num_grid_x);
Z = weight;
Z = 0.2*weightScale*ones(num_grid_y, num_grid_x);% uniformly important...


%%% importance change parmeters
delta_increase = 0.01;
delta_decrease = 1;
