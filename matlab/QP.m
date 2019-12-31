function [u_opt] = QP(xi,yi,u_nom,fieldInfo,persistInfo,Perception,perceptionInfo)
global dhdt 

dhdt = 1;
matlab = 1;






coef = zeros(1,2);

%% for field constraint

%%% \partial x,y?ｿｽﾌ計?ｿｽZ?ｿｽﾉゑｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽC?ｿｽ?ｿｽ?ｿｽﾊの係?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽﾉ計?ｿｽZ
pos = fieldInfo.pos;
theta = fieldInfo.theta;
pnorm = fieldInfo.norm;
width = fieldInfo.width;
hx = fieldInfo.hx;

coef(1) = -pnorm*(((xi-pos(1))*cos(theta)...
    +(yi-pos(2))*sin(theta))/width(1)).^(pnorm-1);    
coef(2) = -pnorm*((-(xi-pos(1))*sin(theta)...
    +(yi-pos(2))*cos(theta))/width(2)).^(pnorm-1);

dh_field = [coef(1)*cos(theta)/width(1)+coef(2)*sin(theta)/width(2);...
    coef(1)*sin(theta)/width(1)+coef(2)*cos(theta)/width(2)];
hx_field = (hx(xi,yi));

% %%% \partial x,y?ｿｽﾌ計?ｿｽZ?ｿｽﾉゑｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽC?ｿｽ?ｿｽ?ｿｽﾊの係?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽ?ｿｽﾉ計?ｿｽZ
% pos = softInfo.pos;
% theta = softInfo.theta;
% pnorm = softInfo.norm;
% width = softInfo.width;
% hx = softInfo.hx;
% 
% coef(1) = -pnorm*(((xi-pos(1))*cos(theta)...
%     +(yi-pos(2))*sin(theta))/width(1)).^(pnorm-1);
% coef(2) = -pnorm*((-(xi-pos(1))*sin(theta)...
%     +(yi-pos(2))*cos(theta))/width(2)).^(pnorm-1);
% 
% dh_soft = [coef(1)*cos(theta)/width(1)+coef(2)*sin(theta)/width(2);...
%     coef(1)*sin(theta)/width(1)+coef(2)*cos(theta)/width(2)];
% hx_soft = (hx(xi,yi));
if Perception
    pos = fieldInfo.pos;
    theta = fieldInfo.theta;
    pnorm = fieldInfo.norm;
    width = fieldInfo.width;
    hx = fieldInfo.hx;

    coef(1) = -pnorm*(((xi-pos(1))*cos(theta)...
                  +(yi-pos(2))*sin(theta))/width(1)).^(pnorm-1);    
    coef(2) = -pnorm*((-(xi-pos(1))*sin(theta)...
        +(yi-pos(2))*cos(theta))/width(2)).^(pnorm-1);

    dh_soft = [coef(1)*cos(theta)/width(1)+coef(2)*sin(theta)/width(2);...
        coef(1)*sin(theta)/width(1)+coef(2)*cos(theta)/width(2)];
    hx_soft = (hx(xi,yi));
else
    dh_soft = persistInfo.dhx';
    if dhdt
        hx_soft = persistInfo.hx'+persistInfo.dht';
    else
        hx_soft = persistInfo.hx';
    end
end
% winf = -1000;
% wsup = 1000;
% epsilon = 1/1000000000;
% uinf = -1000*[1 1];
% usup = 1000*[1 1];

% A = [-dh_hard -dh_soft;
%      0 -norm(dh_soft)*10]'
A = [-dh_field -dh_soft;
     0 -1]';
B = []';
C = []';
D = [-hx_field -hx_soft]';


gQ = sparse(diag([1 1 100])); % 最適化重視
% gQ = sparse(diag([1 1 50])); % ぶつからない重視

% gc = [-u_nom; -1; 0; zeros(2,1)];

gc = [0 0 0];
gq = [A];

%%%%%%%% 解けないときは止まる
% gQ = sparse(diag([1 1 0]));
% gc = [0 0 1];
%%%%%%%%
% gq
% -D


% Qc = sparse(diag([1 1 0]));
% qc = [0 0 0]';
vmax = 0.4;

% ub = [vmax vmax inf];
% lb = [-vmax -vmax -inf];
ub = [vmax; vmax; inf];
lb = -ub;
%% solve by matlab
if matlab
        options = optimoptions('quadprog', 'Display', 'off');
        try
            % adjusting the way of expression of matrix to gurobi
            % -gq x < -D
            u_opt = quadprog(2*gQ,gc,gq,-D, [], [], lb, ub, [], options);
            if isempty(u_opt)
                warning('u_opt EMPTY');
                u_opt = [0;0;0];
            end
        catch
            warning('QuadProg ERROR');
            u_opt = [0;0;0];
        end
%% solve by gurobi
else

    % min x^T gQ x+gc^T x
    % s.t. gq x < rhs

    model.Q = gQ;
    % model.obj = gc;
    model.modelsense = 'min';

    % Au + B\delta + Cz + D < 0
    model.A = sparse(gq);
    model.rhs = -D;
    model.sense = '<'; % '< 'means less or equal

    model.vtype = 'CCC';
    model.lb = lb;
    model.ub = ub;

    % model.quadcon.Qc = Qc;
    % model.quadcon.q = qc;
    % model.quadcon.rhs = vmax;

    params.OutputFlag = 0;
    result = gurobi(model,params);
    u_opt = result.x;
end

end