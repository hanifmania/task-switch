clear all
close all

addpath('../grb')
%% problem setting
% min_{u,\delta,z} |u-unom|^2
% \delta = 1 iff hx>=0
% z = \delta * u


unom = 7;
hx = -0.0001;
hinf = -100;
hsup = 100;
epsilon = 1/100000;
uinf = -100;
usup = 100;

A = [zeros(1,1) zeros(1,1) zeros(1,1) zeros(1,1) eye(1,1) -eye(1,1)]';
B = [-hinf -hsup-epsilon uinf' -usup' usup' -uinf']';
C = [zeros(1,1) zeros(1,1) -eye(1,1) eye(1,1) -eye(1,1) eye(1,1)]';
D = [-hx+hinf hx+epsilon zeros(1,1)' zeros(1,1)' -usup' uinf']';

gQ = sparse([1 0 0; 0 0 0; 0 0 0]);
gc = [-2*unom;0;0];
gq = [A B C];

%% solve by gurobi
% Au + B\delta + Cz + D < 0
model.Q = gQ;
model.obj = gc;
model.modelsense = 'min';

model.A = sparse(gq);
model.rhs = -D;
model.sense = '<'; % '< 'means less or equal

model.vtype = 'CBC';
model.ub = [inf inf inf];
model.lb = [-inf -inf -inf];

result = gurobi(model);
result.x