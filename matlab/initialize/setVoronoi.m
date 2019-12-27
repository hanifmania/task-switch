%%% importance change parmeters
delta_increase = 0.001;
delta_decrease = 0.05;
perception_increase = 0.1;


Region = false(mesh_acc(2),mesh_acc(1),AgentNum);%x方向が列，y方向が行なのでmesh_accびindexが�??にな�?
cent = zeros(AgentNum,2);
Monitored = false(mesh_acc(2),mesh_acc(1),AgentNum);
Perception = zeros(1,AgentNum);