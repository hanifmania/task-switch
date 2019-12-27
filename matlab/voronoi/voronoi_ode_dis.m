function [ux,uy,Voronoi,CBF] = voronoi_ode_dis(x, y, phi, persist)
    global R % neighbor ode
    global X Y mesh_acc pointDense
    persistent AgentNum
    global delta_increase delta_decrease perception_increase
    global goalJ T
    global k b
    
    if isempty(AgentNum)
        AgentNum = length(x);
        % using sparse is one of the option...
        %x direction is Column, y is Row,-> mesh_acc and index is inverse
        Region = false(mesh_acc(2),mesh_acc(1),AgentNum);
        neighbor = false(AgentNum);
        cent = zeros(AgentNum,2);
        mass = zeros(AgentNum,1);
        expand = zeros(AgentNum,2);
        outside = false(mesh_acc(2),mesh_acc(1),AgentNum);
        arc = false(mesh_acc(2),mesh_acc(1),AgentNum);
%         CBF.dht = 0;
%         CBF.dhx = zeros(1,2);
%         CBF.hx = 0;
    end
    

    
    
    for i=1:length(x)
        Region(:,:,i) = ((X-x(i)).^2+(Y-y(i)).^2<R^2);% true for points inside of radius R
        neighbor = (x-x(i)).^2+(y-y(i)).^2<(2*R)^2;% true for agents inside radius 2R 

        neighbor(i) = false;% omit "myself" from neighbor
    
        neighbor_x = x(neighbor);
        neighbor_y = y(neighbor);

        % vector of index which elements is NON-zero in Region
        % ##todo## delete "Region_id" (find function is necessary?)
        Region_id = find(Region(:,:,i));

        % distance between all points in Region and agent i
        distance_i = (X(Region_id)-x(i)).^2+(Y(Region_id)-y(i)).^2;

        % initialize: for all points, neaest agent is i
        i_far_flag = zeros(size(Region_id));

        % for each agent j in neighbor...
        for j=1:length(neighbor_x)
            % distance between all points in Region and agent i
            distance_j = (X(Region_id)-neighbor_x(j)).^2+(Y(Region_id)-neighbor_y(j)).^2;

            % flag: if i is NOT neaest
            i_far_flag = i_far_flag+(distance_i>=distance_j);
        end

        % vector of index for point whose neaest agent is i
        i_near_index = ~logical(i_far_flag);
        Voronoi_Region_id = Region_id(i_near_index);

        % matrix whose elements are true for only neaest point
        i_voronoi_region = false(size(Region,1),size(Region,2));
        i_voronoi_region(Voronoi_Region_id) = true;

        % over write Region
        Region(:,:,i) = i_voronoi_region;

        % based on Region, gather the coordinate of points whose neaest agent is i 
        voronoi_x = X(Region(:,:,i));
        voronoi_y = Y(Region(:,:,i));
        
        % times weight for coordinate
        weighted_x = voronoi_x'*phi(Region(:,:,i));
        weighted_y = voronoi_y'*phi(Region(:,:,i));
       
        % calculate mass
        mass(i) = sum(sum(phi(Region(:,:,i))*pointDense));
        
        % calculate central
        cent(i,:) = [weighted_x, weighted_y]*pointDense/mass(i);
        
    end

    
    gain = 1;
    Ji = getVoronoiEval(x,y,Region,phi);
    for i=1:length(x)
        CBF(i).hx = gain*(Ji(i)-goalJ/AgentNum);
    end
    % calculate edge importance (for persistent coverage)
    if persist
        allRegion = any(Region,3);
        for i=1:length(x)
            outside(:,:,i) = ((X-x(i)).^2+(Y-y(i)).^2<(R+0.1*R)^2);% inside of cicle (radius = R+0.1*R)
            arc(:,:,i) = ~allRegion & outside(:,:,i);% delete region if it's someone's region
            onEdge_x = X(arc(:,:,i));
            onEdge_y = Y(arc(:,:,i));
            onEdge_z = phi(arc(:,:,i));
            distance = [onEdge_x-x(i) onEdge_y-y(i)];
            expand(i,:) = (R^2+b)*pointDense*[(distance(:,1)./(vecnorm(distance,2,2)))'*onEdge_z...
                         (distance(:,2)./(vecnorm(distance,2,2)))'*onEdge_z];
            CBF(i).dhx = 2*mass(i)*(cent(i,:)-[x(i),y(i)])-expand(i,:);
%             CBF.dhx(i,:) = 2*(cent(i,:)-[x(i),y(i)])-expand(i,:)/mass(i);
            
            
%             nom = 2*mass(i)*(cent(i,:)-[x(i),y(i)])
%             ex = expand(i,:)
%             CBF.dht(i,:) = (((X(Region(:,:,i))-x(i)).^2+(Y(Region(:,:,i))-y(i)).^2)'...
%                             *delta_decrease*phi(Region(:,:,i))*pointDense...
%                             +b*delta_increase*((sum(sum(1-phi)))*pointDense/AgentNum...
%                             -(sum(sum(1-phi(Region(:,:,i)))))*pointDense))/mass(i);
            CBF(i).dht = (((X(Region(:,:,i))-x(i)).^2+(Y(Region(:,:,i))-y(i)).^2)'...
                            *delta_decrease*phi(Region(:,:,i))*pointDense...
                            +b*delta_increase*((sum(sum(1-phi)))*pointDense/AgentNum...
                            -(sum(sum(1-phi(Region(:,:,i)))))*pointDense));
            
%             CBF.dht(i,:) = b*mesh_acc(1)*mesh_acc(2)*delta_increase/AgentNum...
%                             -b*sum(sum(Region(:,:,i)))*delta_increase;
        end
    else
        expand = zeros(AgentNum,2);
    end
    
    % targetposition-position
    coverage_ux = (-x+cent(:,1)'-expand(:,1)'/(2*mass(i)));
    coverage_uy = (-y+cent(:,2)'-expand(:,2)'/(2*mass(i)));
    
    ux = k*(coverage_ux);
    uy = k*(coverage_uy);


    Voronoi.Region = Region;
    Voronoi.cent = cent;
    Voronoi.mass = mass;
    
end
