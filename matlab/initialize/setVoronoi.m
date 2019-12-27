%% voronoi settings
Voronoi.Region = false(mesh_acc(2),mesh_acc(1),AgentNum);%x方向が列，y方向が行なのでmesh_accびindexが�??にな�?
Voronoi.cent = zeros(AgentNum,2);
Voronoi.mass = zeros(AgentNum,1);
Monitored = false(mesh_acc(2),mesh_acc(1),AgentNum);


goalJ = -17;

b = -R^2-10;