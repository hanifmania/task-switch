%%%% multiple Drones charging coverage%%%%

clc
clear all
close all
clear java

javaaddpath('jars/org.eclipse.paho.client.mqttv3-1.2.0.jar')
javaaddpath('jars/Subscriber.jar')
addpath(genpath(pwd))
addpath('functions_dis')

global AGENT_NUM
global k % ode
global xlimit ylimit

% for object avoid
global OBJECT_NUM
global drone_r
global theta_obj norm_obj xwidth_obj ywidth_obj f_obj pos_obj
global theta_fie norm_fie xwidth_fie ywidth_fie f_fie pos_fie
global plot_obj
global plot_fie

global hx_o hx_f hx_c

% for charging
global plot_charge_x plot_charge_y
global radius_charge Emin Kc k_charge CBF_fixed
global Echarge

% for drone control 
global b1 c1 Kp1 Ki1 Kd1
global b0 c0 Kp0 Ki0 Kd0

% for coverage
global xbox ybox % local_Q plot
global R % neighbor ode
global X Y mesh_acc % mesh grid
global cover_offset
global monitor_x monitor_y
global delta_decrease delta_increase perception_increase
global object_func


%% Mode setting
real = 1;

% Plot in matlab or ROS.
matlab_plot = 1;
crazyflie = 0;
bebop = 1;

% no_ctrl = 0;


%% Initial setting



% Drone number list for experiment. Ex. [1, 3, 6]
drone_list = [1];
AGENT_NUM = size(drone_list,2);

% Drone r for collision avoidance
% CBF force them to keep 2*r distance
if bebop
    drone_r = 0.3;
elseif crazyflie
    drone_r = 0.2;
end

% Set desired height of drone.(only for bebop2)
z_ref = 1.2;
% Set height thresh hold for taking off (only for bebop2)
z_thresh = 0.7;


% Define and fill a rectangular area in the plane
dd = 0.25;
xlimit = [-1.7 2.0];
ylimit = [-1.2 1.4];
xbox = xlimit([1 2 2 1 1]);
ybox = ylimit([1 1 2 2 1]);


% set initial position
x = mean(xlimit) + (xlimit(2)-xlimit(1)).*(gallery('uniformdata',[1 AGENT_NUM], 0)-0.5);
y = mean(ylimit) + (ylimit(2)-ylimit(1)).*(gallery('uniformdata',[1 AGENT_NUM], 1)-0.5);

% Set Initial Position of Agents (only for simulation)
if ~real
%     x = [-1.2, -1.2, -1.2];
%     y = [0, drone_r*2, -drone_r*2];
%     z = [1, 1, 1];
      x = [-1.2, -1.2];
      y = [drone_r*2, -drone_r*2];
      z = [1, 1];

end

% % drone private area (fixed circle)
r_sa = drone_r;
theta = linspace(0, 2*pi);
size_th = size(theta);
x_plot = zeros(size_th(2),AGENT_NUM);
y_plot = zeros(size_th(2),AGENT_NUM);
ref = zeros(AGENT_NUM,2);

%% coverage setting 
mesh_acc = [200 150];% メッシュ精度:[n m]でx方向n点y方向m点に分割

cover_offset = 0;
xgrid = linspace(xlimit(1),xlimit(2)-cover_offset,mesh_acc(1));
ygrid = linspace(ylimit(1),ylimit(2),mesh_acc(2));

[X,Y] = meshgrid(xgrid,ygrid);

num_grid_x = mesh_acc(1);
num_grid_y = mesh_acc(2);

% weight
mu = [0 0];
sigma = [0.25 0.3; 0.3 1];
xy = [X(:) Y(:)];
W = mvnpdf(xy,mu,sigma);
W = reshape(W,num_grid_y,num_grid_x);
Z = 0.5*ones(num_grid_y, num_grid_x);
% Z = W;


% Set Monitoring area (fixed circle)
% virtual_c = (r_sa+(Y(1)-Y(2))*1.1)/r_sa;
% x_re = reshape(cos(theta)*r_sa, numel(cos(theta)), 1);
% y_re = reshape(sin(theta)*r_sa, numel(sin(theta)), 1);

% R = 10.0; % ball radius. search radius = 2R.
R = 0.5;% bebop z = 1.2 --> 1.8*1.0 picture


% IN_v = false(num_grid_y, num_grid_x, AGENT_NUM);
% IN_b = false(num_grid_y, num_grid_x, AGENT_NUM);
% IN_s = false(num_grid_y, num_grid_x, AGENT_NUM);
% IN_large = false(num_grid_y, num_grid_x, AGENT_NUM);
% ON_e = false(num_grid_y, num_grid_x, AGENT_NUM);
mass = zeros(AGENT_NUM,1);
cent = zeros(AGENT_NUM,2);
% on_edge = zeros(AGENT_NUM,2);




% Set weight decay and increase rate.
% delta_increase = 0.01;
% delta_decrease = 0.01;
delta_increase = 0.001;
delta_decrease = 0.05;
perception_increase = 0.01;

% 何かを発見した時のフラグ
Perception = zeros(AGENT_NUM,1);


% 各エージェントのモニタリング領域
Region = false(mesh_acc(2),mesh_acc(1),AGENT_NUM);%x方向が列，y方向が行なのでmesh_accびindexが逆になる

%モニタリングの外側を見る用
outside = false(mesh_acc(2),mesh_acc(1),AGENT_NUM);
arc = false(mesh_acc(2),mesh_acc(1),AGENT_NUM);

%% Charging settings
% charging and consumption speed
Kc = 0;
% initial Energy level
E = [4500 3500 2500];
Echarge = 4500;
Emin = 1500;

CBF_fixed = 1;

% estimated speed for drone returning to station.
k_charge = 0.2;

% Set charging station position
radius_charge = 0;
station_distance = (sum(abs(ylimit)) - 2*radius_charge)/(AGENT_NUM-1);

pos_charge = zeros(2,AGENT_NUM);
for i=1:AGENT_NUM
    pos_charge(:,i) = [xlimit(2) - 0.3; ylimit(2) - radius_charge - station_distance*(i-1)];
end
for i=1:AGENT_NUM
    plot_charge_x = [plot_charge_x; radius_charge*sin(theta)+pos_charge(1,i)];
    plot_charge_y = [plot_charge_y; radius_charge*cos(theta)+pos_charge(2,i)];
end

land_flags = zeros(1, AGENT_NUM);

%% CBF
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Field limitation

pos_fie = [sum(xlimit)/2; sum(ylimit)/2];   %中心
theta_fie = [0];%回転角
norm_fie = [10]; %偶数
xwidth_fie = [(xlimit(2)-xlimit(1))/2];%x横幅
ywidth_fie = [(ylimit(2)-ylimit(1))/2];%y横幅


%%%norm=2で楕円，でかくするほど四角形
f_fie = @(x,y) -(((x-pos_fie(1))*cos(theta_fie)+...
    (y-pos_fie(2))*sin(theta_fie))/xwidth_fie).^norm_fie...
    -((-(x-pos_fie(1))*sin(theta_fie)...
    +(y-pos_fie(2))*cos(theta_fie))/ywidth_fie).^norm_fie+1;
%%%
%%%プロット用
%%%毎ステップfimplicitするのはやばそうだから先に
%%%描画用のプロットデータだけ先にとっておく
temp_plot = fimplicit(f_fie,'XRange',xlimit*1.1,'YRange',ylimit*1.1,'visible','off');
plot_fie_.X = temp_plot.XData;
plot_fie_.Y = temp_plot.YData;
plot_fie = [plot_fie plot_fie_];



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Obstacle Avoidance
% pos_obj = [sum(xlimit)/2; sum(ylimit)];   %中心
% pos_obj = [mean(pos_charge(1,1:2))-0.3 mean(pos_charge(1,2:3))-0.3; mean(pos_charge(2,1:2)) mean(pos_charge(2,2:3))];   %中心[x x x...;y y y...]
theta_obj = [0 0];%回転角(それぞれの障害物に対して設定)
norm_obj = [2 2]; %偶数(それぞれの障害物に対して設定)
xwidth_obj = [0.4 0.4];%x横幅
ywidth_obj = (station_distance/2-radius_charge)*[1 1];%y横幅(それぞれの障害物に対して設定)
% OBJECT_NUM = size(norm_obj, 2);
OBJECT_NUM = 0;
f_obj= {};

for i=1:OBJECT_NUM
    %%%norm=2で楕円，でかくするほど四角形
    f_obj_ = @(x,y) -(((x-pos_obj(1,i))*cos(theta_obj(i))+...
        (y-pos_obj(2,i))*sin(theta_obj(i)))/xwidth_obj(i)).^norm_obj(i)...
        -((-(x-pos_obj(1,i))*sin(theta_obj(i))...
        +(y-pos_obj(2,i))*cos(theta_obj(i)))/ywidth_obj(i)).^norm_obj(i)+1;
    %%%
    f_obj{i} = {f_obj_};
    

%%%プロット用
%%%毎ステップfimplicitするのはやばそうだから先に
%%%描画用のプロットデータだけ先にとっておく
    temp_plot = fimplicit(f_obj{i},'XRange',xlimit,'YRange',ylimit,'visible','off');
    plot_obj_.X = temp_plot.XData;
    plot_obj_.Y = temp_plot.YData;
    plot_obj = [plot_obj plot_obj_];
end
%%


% Set control gain
if real
    k = 0.2;
else
    k = 0.6;
end
% Set control gain for height (only for bebop2)
Kz = 0.3;

% PIDcontroller
% mode1 is for xy. mode0 is for z
Kp1 = 1.2;
Ki1 = 0.14;
Kd1 = 0.10;
b1 = 1;
c1 = 1;


% Kp0 = 1.9;
% Ki0 = 0.37;
% Kd0 = 0.10;
% b0 = 1;
% c0 = 1;
Kp0 = 1.2;
Ki0 = 0.14;
Kd0 = 0.10;
b0 = 1;
c0 = 1;



%% MQTT setting

mqttinterface = MqttInterface(['matlab_mqtt_node_',num2str(datenum(datetime('now')))], '127.0.0.1', 1883);

joy_topic = 'joy';
sub_joy = {};
sub_joy{1} = joy_topic;
mqttinterface.add_subscriber(sub_joy{1});

if real    
    if bebop
%%%%%%% Change here to corresponding topic name without agent number.
%%%%%%% Topic names for experiment with bebop2
        vel_topic = 'vel0';
        pose_topic = 'pose0';
        takeoff_topic = 'takeoff0';
        land_topic = 'land0';
        obj_topic = 'objects0';
    elseif crazyflie
%%%%%%% Topic names for experiment with crazyflie
        vel_topic = 'vel'; 
        pose_topic = 'pose';
        takeoff_topic = 'takeoff';
        land_topic = 'land';
    end

 %%%%% Pose topic name for simulation
else
    pose_topic = 'pose';
end


% If it's real experiment, publish velocity and subscribe poses
if real
    
    pub_vels = {};
    sub_poses = {};
    pub_takeoffs = {};
    pub_lands = {};

    
%     Create str of topic names
    for i=1:AGENT_NUM
        agent_num_str = num2str(drone_list(i));
        pub_vels{i} = strcat(vel_topic, agent_num_str);
        sub_poses{i} = strcat(pose_topic, agent_num_str);
        pub_takeoffs{i} = strcat(takeoff_topic, agent_num_str);
        pub_lands{i} = strcat(land_topic, agent_num_str);
        sub_obj{i} = strcat(obj_topic, agent_num_str);

    end
    
%     Add subscribers and publisher topics to MQTT
    for i=1:AGENT_NUM
        mqttinterface.add_publisher(pub_vels{i});
        mqttinterface.add_subscriber(sub_poses{i});
        mqttinterface.add_subscriber(sub_obj{i});
        if bebop
            mqttinterface.add_publisher(pub_takeoffs{i});
            mqttinterface.add_publisher(pub_lands{i});
            takeoffmsg.data = '';
            landmsg.data = '';
        elseif crazyflie
            mqttinterface.add_publisher(pub_takeoffs{i});
            mqttinterface.add_publisher(pub_lands{i});
            takeoffmsg.data = 1.0;
            landmsg.data = '';
        end
    end
    
    %Create Twist message
    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;
   
% If it's simulation, publish poses
else
    pub_poses = {};
    for i=1:AGENT_NUM
        agent_num_str = num2str(i);
        pub_poses{i} = strcat(pose_topic, agent_num_str);
        mqttinterface.add_publisher(pub_poses{i});
    end

    
%     Create Pose message
    pose_msg.header.frame_id = 'world';
    pose_msg.pose.position.x = 0;
    pose_msg.pose.position.y = 0;
    pose_msg.pose.position.z = 0;
    pose_msg.pose.orientation.x = 0;
    pose_msg.pose.orientation.y = 0;
    pose_msg.pose.orientation.z = 0;
    pose_msg.pose.orientation.w = 1;
end

if ~matlab_plot
    pub_plot = "plot_data";
    mqttinterface.add_publisher(pub_plot);
end


%%%center reference 
ref_plot = "reference";
mqttinterface.add_publisher(ref_plot);




mqttinterface.connect();


%% Send Fixed Data for Plot before starting simulation(experiment).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%   Define all field %%%%%%%%%%%%%%%
if ~matlab_plot
    plot_msg.robot_num = [1 2 3];
    plot_msg.agent_radius = r_sa;
    plot_msg.obstacle_xwidth= xwidth_obj;
    plot_msg.obstacle_ywidth= ywidth_obj;
    for i=1:OBJECT_NUM
        plot_msg.obstacle_positions.poses(i).position.x = pos_obj(1,i);
        plot_msg.obstacle_positions.poses(i).position.y = pos_obj(2,i);
        plot_msg.obstacle_positions.poses(i).position.z = 1;
        plot_msg.obstacle_positions.poses(i).orientation.x = 0;
        plot_msg.obstacle_positions.poses(i).orientation.y = 0;
        plot_msg.obstacle_positions.poses(i).orientation.z = 1;
        plot_msg.obstacle_positions.poses(i).orientation.w = cos(pi/2);
    end
    mqttinterface.send(pub_plot, plot_msg);
end



%% Start simulation or experiment

u_nom = zeros(2, AGENT_NUM);
u_opt = zeros(2, AGENT_NUM);
u_z = zeros(1,AGENT_NUM);
orientation = cell(1,AGENT_NUM);
intstate = zeros(3,AGENT_NUM);%integral for xyz
lastdiff = zeros(3,AGENT_NUM);%derivative for xyz



detectNum = 0;



%%% wait until get non-empty msg
if real
    pose_msg = mqttinterface.receive(sub_poses{1});
    while isempty(pose_msg)
        pose_msg = mqttinterface.receive(sub_poses{1});
    end
end

joy_msg = mqttinterface.receive(sub_joy{1});
while isempty(joy_msg)
    joy_msg = mqttinterface.receive(sub_joy{1});
end


endflag = 0;
t = 0;

t_sim_start = tic;
last_toc = toc(t_sim_start);

object_func = [];


while(~endflag)
    if t == 0
        input('PRESS ENTER to START')
        disp('START!!!!!!!!!')
    end
    samplingtime = toc(t_sim_start)-last_toc;
    last_toc = toc(t_sim_start);
    t = t+samplingtime;

%%%%%%%%%%%%%%%% Get agent positions  %%%%%%%%%%%%%
    if real
        for i = 1:AGENT_NUM
%          Receive PoseStamped message (sent from vrpn_client)
           pose_msg = mqttinterface.receive(sub_poses{i});
           x(i) = pose_msg.pose.position.x;
           y(i) = pose_msg.pose.position.y;
           z(i) = pose_msg.pose.position.z;
           orientation{i} = pose_msg.pose.orientation;
           detection = mqttinterface.receive(sub_obj{i});
        end
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    detectNum = size(detection.detections,1);
    if detectNum
        for cnt=1:detectNum
            detections = detection.detections(cnt);
            findit = [detections.results.score detections.results.id]
        end
    end


%%%%%%%%%%%%%%%% JoyStick read %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    joy_msg = mqttinterface.receive(sub_joy{1});
    Lbutton = joy_msg.buttons(5);
    Rbutton = joy_msg.buttons(6);
    
    endflag = joy_msg.buttons(7);
    

%%%%%%%%%%%%%%%% coverage %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    Perception = [Lbutton Rbutton];
    
    
    Z = weight(x,y,Z,Perception);% フィールド重み更新

    [Region,cent]=voronoi_ode_dis(x,y,Z);
    allRegion = any(Region,3);% 各エージェントのRegionを論理和したもの
    
    b_ = 0.005;
    b = b_+R^2;    
    
    objfunc_second = b*sum(Z(~allRegion));
    objfunc_first = zeros(1,AGENT_NUM);
    
    for i = 1:AGENT_NUM 

%         edge_col = edge_i(:);
%         edge_x = edge_col(1:2:end);
%         edge_y = edge_col(2:2:end);
%         
% %      CAUTION!! Information reliability and Weight is caluculated
% %      Conversely to the paper!!!!!!!!!
% %      W: Informatin Reliability, Z: Weight
% 
% %         Calculate Weiht and IN_b
%         [Z,IN_b]=weight(Z,IN_b,x,y,x_re,y_re,i);
% %         Set Information reliability
%         W = 1-Z;
%                        
%         IN_large(:,:,i) = inpolygon(grid_x, grid_y, x_re*virtual_c+x(i), y_re*virtual_c+y(i));
%         ON_e = (IN_b~=IN_large);
%         IN_v(:,:,i) = inpolygon(grid_x,grid_y,edge_x,edge_y);
%         ON_e(:,:,i) = IN_v(:,:,i).*ON_e(:,:,i);
%         IN_s(:,:,i) = ( IN_v(:,:,i).*IN_b(:,:,i) );
% 
%         dis = [ grid_x(ON_e(:,:,i))-x(i), grid_y(ON_e(:,:,i))-y(i) ];
%         on_edge(i,:) = sum((dis/norm(dis)).*Z(ON_e(:,:,i)));
%         mass(i) = sum(Z(IN_s(:,:,i)));
%         cent(i,:) = sum( [ grid_x(IN_s(:,:,i)), grid_y(IN_s(:,:,i)) ].*Z(IN_s(:,:,i)) ) ./mass(i);
%         ref(i,:) = cent(i,:) + b*on_edge(i,:);
%         object_func(i,count) = sum( sum(([grid_x(IN(:,:,i))-x(i), grid_y(IN(:,:,i))-y(i)] .^2),2) .*Z(IN(:,:,i)) );
        
        objfunc_first = Z(Region(:,:,i))'*sqrt((X(Region(:,:,i))-x(i)).^2 + (Y(Region(:,:,i))-y(i)).^2);

        
        

        % Regionの「円弧」部分について，その外側の領域の重要度を調べる
        outside(:,:,i) = ((X-x(i)).^2+(Y-y(i)).^2<(R+0.1)^2);% 半径R+0.1の円の内部
        arc(:,:,i) = ~allRegion.*outside(:,:,i);% 誰かの領域だったらその部分を削除
        onEdge_x = X(arc(:,:,i));
        onEdge_y = Y(arc(:,:,i));
        onEdge_z = Z(arc(:,:,i));
        
        % 円弧について，その法線ベクトルを足し合わせる
        % エッジを共有している部分は円弧ではないため無視される，
        % よってエージェント間は離れる方向へ動く．
        dis = [onEdge_x-x(i) onEdge_y-y(i)];
        expand = [sum((dis(:,1)./(vecnorm(dis,2,2))).*onEdge_z,'all') ...
                    sum((dis(:,2)./(vecnorm(dis,2,2))).*onEdge_z,'all')];
        
        % 要するにcent+b_*expand
        % 論文と表現を合わせるためにこの形に
        ref(i,:) = cent(i,:)+(b-R^2)*expand;

%%%%%%%%%%%%%%% coverage calc. end %%%%%%%%%%%%%%%%%%%%%%%%%%

      
        [u_nom(1,i), intstate(1,i), lastdiff(1,i)] = ...
            PID2dof(x(i),ref(i,1),intstate(1,i),lastdiff(1,i),samplingtime,1);
        [u_nom(2,i), intstate(2,i), lastdiff(2,i)] = ...
            PID2dof(y(i),ref(i,2),intstate(2,i),lastdiff(2,i),samplingtime,1);
        
        u_nom(:,i) = [- k*(x(i)-ref(i,1)); - k*(y(i)-ref(i,2))]; % dx = v * dT  
        [u_nom(1,i), u_nom(2,i)] = sat(k*u_nom(:,i));
        
        
        x_others = x;%xyz
        y_others = y;
        x_others(i) = [];
        y_others(i) = [];
        [u_opt(:,i)] = QP2(x(i), y(i), u_nom(:,i), x_others, y_others, E(i), pos_charge(:,i));
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if real
            quat = [orientation{i}.w orientation{i}.x orientation{i}.y orientation{i}.z];
            rotm = quat2rotm(quat);
            body_vel = rotm' * [u_opt(:,i); 0];
            u_opt(:,i) = body_vel(1:2);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        
        
        
        
%         u_z(i) = Kz*(z_ref - z(i));        

        u_z(i) = ...
            PID2dof(z(i),z_ref,intstate(3,i),lastdiff(3,i),samplingtime,0);

        
        

        
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%charging
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % Within Charging Station and Energy Level is low, send land.
        if sqrt((x(i)-pos_charge(1,i))^2 + (y(i)-pos_charge(2,i))^2) < radius_charge  &&  E(i) <= Emin + (Echarge - Emin)/3 && land_flags(i) == 0
            E(i) = E(i) + 5*Kc*samplingtime;
            u_opt(:,i) = [0;0];
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if real
                if bebop
                    landmsg.data = 'land';
                    mqttinterface.send(pub_lands{i}, landmsg);   
                elseif crazyflie
                    mqttinterface.send(pub_lands{i}, landmsg);
                end
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            land_flags(i) = 1
         
%            Landing in charging station, not enough charged. Do nothing
        elseif land_flags(i) == 1 && E(i) <= Echarge
            E(i) = E(i) + 5*Kc*samplingtime;
            u_opt(:,i) = [0;0];
            
%         Landing in charging station, enough charged. Send takeoff.
        elseif land_flags(i) == 1 && E(i) >= Echarge
            
    
           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
           if real
                if bebop
                    takeoffmsg.data = 'takeoff';
                    mqttinterface.send(pub_takeoffs{i}, takeoffmsg);
                elseif crazyflie
                    mqttinterface.send(pub_takeoffs{i}, takeoffmsg);
                end
           end
           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
           
           land_flags(i) = 0
            
        % Out of Charging Station OR within charging station with enough energy, send velocity input.
        else
            E(i) = E(i) - Kc*samplingtime;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %       Send command velocity only if height is high enough.
            if real
                if z(i) > z_thresh
                    vel_msg.linear.x = u_opt(1,i);
                    vel_msg.linear.y = u_opt(2,i);
                    if abs(z(i) - z_ref) < 0.1
                        vel_msg.linear.z = 0;
                    else
                        vel_msg.linear.z = u_z(i);
                    end
                end
                if bebop
                    takeoffmsg.data = '';
                    landmsg.data = '';
                    mqttinterface.send(pub_takeoffs{i}, takeoffmsg);
                    mqttinterface.send(pub_lands{i}, landmsg);
                end
                mqttinterface.send(pub_vels{i}, vel_msg);
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
        end
        
        
        
        
        %%%%%%%%%%% Plot setting for monitoring circle %%%%%%%%%%%%%
        x_plot(:,i) = r_sa*cos(theta) + x(i);
        y_plot(:,i) = r_sa*sin(theta) + y(i);
    end
    
%%%%%%%%%% Update the position to next step %%%%%%%%%%%%%%%%
    
    if ~real
        for i=1:AGENT_NUM
            x(i) = x(i) + u_opt(1,i) * samplingtime;
            y(i) = y(i) + u_opt(2,i) * samplingtime;
            z(i) = z(i) + u_z(i) * samplingtime;
        end
    end

    %%%%%%%%% Plot %%%%%%%%%%%
    if matlab_plot
        voronoi_plot_mine2(x, y, x_plot, y_plot, E, ref', Region, Z);
%         voronoi_plot_mine2(x, y, x_plot, y_plot, E, cent', Region, Z);
    else
%         If it's simulation, publish poses
    if ~real
        for i=1:AGENT_NUM
            pose_msg.pose.position.x = x(i);
            pose_msg.pose.position.y = y(i);
            pose_msg.pose.position.z = 1;

            mqttinterface.send(pub_poses{i}, pose_msg);
        end
    end
%         ref_msg.header.frame_id = 'world';
%         ref_msg.pose.position.x = formcenter(1);
%         ref_msg.pose.position.y = formcenter(2);
%         ref_msg.pose.position.z = 1;
%         ref_msg.pose.orientation.x = 0;
%         ref_msg.pose.orientation.y = 0;
%         ref_msg.pose.orientation.z = sin(bias_theta/2);
%         ref_msg.pose.orientation.w = cos(bias_theta/2);
%         mqttinterface.send(ref_plot, ref_msg);
    end
    
    if object_func        
        if (t-object_func(end,1))>0.1
            object_func = [object_func;t sum(objfunc_first)+objfunc_second];
            CBF = [CBF; t hx_f];
        end
    else
        object_func = [t sum(objfunc_first)+objfunc_second];
        CBF = [t hx_f];
    end
% image hold
%     stp_mem = 10;
%     if rem(round(stp),stp_mem) == 0 || stp == 1
%         disp('--- SAVE IMAGE ---');
%         rootname = 'figure/pos_';
%         extension = '.png';
%         filename = [rootname, num2str(stp), extension];
%         saveas(gcf,filename);
%     end

%    sampletime(round(stp)) = toc;
end


%% End of experiment
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if real
    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    for j=1:5
        for i=1:AGENT_NUM
            mqttinterface.send(pub_vels{i},vel_msg)
        end
    end    
end
disp('END!!!!!!!!!!!!')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Plot after simulation

figure;plot(CBF(:,1),CBF(:,2));




% % plot result of h_charging
% figure
% maximum = max(max(CBF_h_cra));
% margin = maximum*0.1;   
% CBF_h_cra = reshape(CBF_h_cra,AGENT_NUM,[]);
% for i=1:AGENT_NUM
% 
% %    plot(0:T:(step-1)*T,CBF_h(i,:),'LineWidth',2);
%     plot(0:dt:(num_step)*dt, CBF_h_cra(i,:),'LineWidth',2);
%     xlim([0 num_step*dt]);
%     ylim([0-margin maximum+margin]);
%     hold on;
% end
% zero_Line = refline(0);
% zero_Line.Color = 'm';
% zero_Line.LineWidth = 3;
% zero_Line.LineStyle = ':';
% title('Result of h charging')
% legend('1','2')
% % save('save/data_h_cha.mat', 'CBF_h_cha');
% 
% 
% % plot result of h(obs_avoidance)
% for i=1:OBJECT_NUM
%     h = CBF_h_obs(:,i);
%     h = reshape(h,AGENT_NUM,[]);
% 
%     figure
%     maximum = max(max(h));
%     margin = maximum*0.1;   
%     for k=1:AGENT_NUM
%         plot(0:dt:(num_step)*dt, h(k,:), 'LineWidth',2);
%         xlim([0 num_step*dt]);
%         ylim([0-margin maximum+margin]);
%         hold on;
%     end
%     zero_Line = refline(0);
%     zero_Line.Color = 'm';
%     zero_Line.LineWidth = 3;
%     zero_Line.LineStyle = ':';
%     
%     title('Result of h_obs_avoidance')
%     legend('1','2')
%     
%     % data hold
%     rootname = 'save/data_h_obs';
%     extension = '.mat';
%     filename = [rootname, num2str(i), extension];
% %     save(filename, 'h');
% end


% % plot result of energy level
% maximum = max(max(Eplot));
% margin = maximum*0.1;   
% figure
% for i=1:AGENT_NUM
%     
%     plot(0:dt:(num_step)*dt, Eplot(i,:), 'LineWidth', 2);
%     xlim([0 num_step*dt]);
%     ylim([Emin-100 Echarge+margin]);
%     hold on;
%     
% end
% Emin_Line = refline(0,Emin);
% Emin_Line.Color = 'm';
% Emin_Line.LineWidth = 3;
% Emin_Line.LineStyle = ':';
% Echarge_Line = refline(0,Echarge);
% Echarge_Line.Color = 'm';
% Echarge_Line.LineWidth = 3;
% Echarge_Line.LineStyle = ':';
% title('Energy level')
% legend('1','2')
% % save('save/data_E.mat', 'Eplot');