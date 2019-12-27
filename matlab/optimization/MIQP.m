function [u_opt] = MIQP(xi,yi,u_nom,targetInfo,flagInfo)

ConstraintNum = 1;
dh_con = zeros(2,ConstraintNum);
hx_con = zeros(1,ConstraintNum);
coef = zeros(1,2);


for i=1:ConstraintNum
    %%% \partial x,y�̌v�Z�ɂ�����C���ʂ̌W��������Ɍv�Z
    pos = targetInfo.pos;
    theta = targetInfo.theta;
    norm = targetInfo.norm;
    width = targetInfo.width;
    hx = targetInfo.hx;
    
    coef(1) = -norm*(((xi-pos(1))*cos(theta)...
        +(yi-pos(2))*sin(theta))/width(1)).^(norm-1);
    coef(2) = -norm*((-(xi-pos(1))*sin(theta)...
        +(yi-pos(2))*cos(theta))/width(2)).^(norm-1);
    
    dh_con(:,i) = [coef(1)*cos(theta)/width(1)+coef(2)*sin(theta)/width(2);...
        coef(1)*sin(theta)/width(1)+coef(2)*cos(theta)/width(2)];
    hx_con = (hx(xi,yi));
end


%%% \partial x,y�̌v�Z�ɂ�����C���ʂ̌W��������Ɍv�Z
pos = flagInfo.pos;
theta = flagInfo.theta;
norm = flagInfo.norm;
width = flagInfo.width;
hx = flagInfo.hx;

coef(1) = -norm*(((xi-pos(1))*cos(theta)...
    +(yi-pos(2))*sin(theta))/width(1)).^(norm-1);
coef(2) = -norm*((-(xi-pos(1))*sin(theta)...
    +(yi-pos(2))*cos(theta))/width(2)).^(norm-1);

d_h_f = [coef(1)*cos(theta)/width(1)+coef(2)*sin(theta)/width(2);...
    coef(1)*sin(theta)/width(1)+coef(2)*cos(theta)/width(2)];
h_f = (hx(xi,yi));


hinf = -100000;
hsup = 100000;
epsilon = max(1/1000000000,hx_con(1)/10000);
uinf = [-10000;-10000];
usup = [10000;10000];

A = [zeros(2,1) zeros(2,1) zeros(2) zeros(2) eye(2) -eye(2) zeros(2,1) zeros(2,ConstraintNum)]';
B = [epsilon-hinf -hsup uinf' -usup' usup' -uinf' -h_f -hx_con]';
C = [zeros(2,1) zeros(2,1) -eye(2) eye(2) -eye(2) eye(2) -d_h_f -dh_con]';
D = [-h_f+hinf h_f zeros(2,1)' zeros(2,1)' -usup' uinf' zeros(1,ConstraintNum+1)]';

gQ = sparse([0.5*eye(2) zeros(2,1) zeros(2); zeros(1,5); zeros(2,5)]);
gc = [-u_nom; 0; zeros(2,1)];
gq = [A B C];

%% solve by gurobi
% Au + B\delta + Cz + D < 0
model.Q = gQ;
model.obj = gc;
model.modelsense = 'min';

model.A = sparse(gq);
model.rhs = -D;
model.sense = '<'; % '< 'means less or equal

model.vtype = 'CCBCC';
model.lb = [-inf -inf -inf -inf -inf];
params.OutputFlag = 0;
result = gurobi(model,params);
u_opt = result.x;

end

