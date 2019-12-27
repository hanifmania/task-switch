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


