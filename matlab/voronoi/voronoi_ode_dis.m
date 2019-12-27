function [ux,uy,Region,cent] = voronoi_ode_dis(x, y, phi, k, persist)
    global R % neighbor ode
    global X Y mesh_acc
    persistent AgentNum
    
    if isempty(AgentNum)
        AgentNum = length(x);
        % sparseにしてもよさげ
        Region = false(mesh_acc(2),mesh_acc(1),AgentNum);%x方向が列，y方向が行なのでmesh_accびindexが�??にな�?
        neighbor = false(AgentNum);
        cent = zeros(AgentNum,2);
        if persist
            expand = zeros(AgentNum,2);
            outside = false(mesh_acc(2),mesh_acc(1),AgentNum);
            arc = false(mesh_acc(2),mesh_acc(1),AgentNum);
        end
    end
    
    for i=1:length(x)
        Region(:,:,i) = ((X-x(i)).^2+(Y-y(i)).^2<R^2);% 半径R�?に�?る点のみtrue
        neighbor = (x-x(i)).^2+(y-y(i)).^2<(2*R)^2;% 半�?2R�?に�?るエージェント�?�みtrue 

        neighbor(i) = false;% 自己を隣人に含めな�?
    
        neighbor_x = x(neighbor);
        neighbor_y = y(neighbor);

        % Region�?で非ゼロ要�?のindexを�?�クトル�?
        % 本当�?�findを使�?たくな�?(Region_idとかい�?変数を消したい)
        Region_id = find(Region(:,:,i));

        % エージェン�?iとRegion�?の全点の距離を計�?
        distance_i = (X(Region_id)-x(i)).^2+(Y(Region_id)-y(i)).^2;

        % 全点につ�?て�?も近いエージェントがiであると初期�?
        i_far_flag = zeros(size(Region_id));

        % neighborの�?エージェン�?jにつ�?て...
        for j=1:length(neighbor_x)
            % エージェン�?jとRegion�?の全点の距離を計�?
            distance_j = (X(Region_id)-neighbor_x(j)).^2+(Y(Region_id)-neighbor_y(j)).^2;

            % iのほ�?が遠ければ?��flagを立て�?
            i_far_flag = i_far_flag+(distance_i>=distance_j);
        end

        % iのほ�?が近い点のindexを�?�クトル�?
        i_near_index = ~logical(i_far_flag);
        Voronoi_Region_id = Region_id(i_near_index);

        % iのほ�?が近い点のみtrueが�?�った行�??
        i_voronoi_region = false(size(Region,1),size(Region,2));
        i_voronoi_region(Voronoi_Region_id) = true;

        % Regionを上書�?
        Region(:,:,i) = i_voronoi_region;

        % Regionに基づき，最も近いエージェントがiとなる点群の座標を�?める
        voronoi_x = X(Region(:,:,i));
        voronoi_y = Y(Region(:,:,i));
        
        % 座標に重みをかける
        weighted_x = voronoi_x'*phi(Region(:,:,i));
        weighted_y = voronoi_y'*phi(Region(:,:,i));
        
        % 質�?
        mass = sum(sum(phi(Region(:,:,i))));
        
        % 重�?計�?
        cent(i,:) = [weighted_x, weighted_y]/mass;
    end
    
    % calculate edge importance (for persistent coverage)
    if persist
        allRegion = any(Region,3);
        for i=1:length(x)
            outside(:,:,i) = ((X-x(i)).^2+(Y-y(i)).^2<(R+0.1)^2);% inside of cicle which radius is R+0.1
            arc(:,:,i) = ~allRegion & outside(:,:,i);% delete region if it's someone's region
            onEdge_x = X(arc(:,:,i));
            onEdge_y = Y(arc(:,:,i));
            onEdge_z = phi(arc(:,:,i));
            distance = [onEdge_x-x(i) onEdge_y-y(i)];
            expand(i,:) = [sum(sum((distance(:,1)./(vecnorm(distance,2,2))).*onEdge_z)) ...
                        sum(sum((distance(:,2)./(vecnorm(distance,2,2))).*onEdge_z))];
        end
    else
        expand = zeros(AgentNum,2);
    end
    
    %%%b<=-R^2, matching to cortes's paper
    b_ = 1;
    b = -b_-R^2;  
    ux = k*(-x+cent(:,1)'+(-b-R^2)*expand(:,1)'/mass);
    uy = k*(-y+cent(:,2)'+(-b-R^2)*expand(:,2)'/mass);
    
end

