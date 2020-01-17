clear all
close all
clear java

addpath('voronoi')
addpath('optimization')
addpath('initialize')
addpath('grb')
addpath('functions')
javaaddpath('jars/org.eclipse.paho.client.mqttv3-1.2.0.jar')
javaaddpath('jars/Subscriber.jar')
addpath(genpath(pwd))

global xlimit ylimit % plot
global R % neighbor ode
global X Y mesh_acc % mesh grid
global delta_increase delta_decrease perception_increase
global weightScale T
global b
global pointDense
global k % control gain
global goalJ
global charge
global Echarge
%% Mode setting
real = 1;

% Plot in matlab or ROS.
matlab_plot = 1;
rviz_info_plot = 0;
crazyflie = 0;
bebop = 1;


%% field settings
setField


%% agents settings
setDroneConfig

%% charging settings(energy, station,etc...)
setCharge
%% controller settings
setCtrlConfig

%% voronoi settings
setVoronoi

%% perception settings
setPerception

%% Hard constraint CBF settings
setHardCBF


%% simulation settings
T = 0.1;
simtime = 100;

%% MQTT
setMQTT


%%

orientation = cell(1,AgentNum);

disp('mqtt message waiting...')
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
disp('message received')


endflag = 0;
t = 0;




%%%% for save CBF values%%%%%%%%%%%%%%%%
%%%% time field , charge, persistent, target follow
hxf = zeros(1,AgentNum);
hxt = zeros(1,AgentNum);
hxc = zeros(1,AgentNum);
hxp = zeros(1,AgentNum);
hxc_ = zeros(1,AgentNum);

savefile.fieldCBFvalue = zeros(1,AgentNum);
savefile.chargeCBFvalue = zeros(1,AgentNum);
savefile.persistentCBFvalue = zeros(1,AgentNum);
savefile.targetCBFvalue = zeros(1,AgentNum);
savefile.time = 0;
savefile.energy = E;
savefile.J = zeros(1,AgentNum);
savefile.uX = zeros(1,AgentNum);
savefile.uY = zeros(1,AgentNum);
savefile.w = zeros(1,AgentNum);
savefile.x = zeros(1,AgentNum);
savefile.y = zeros(1,AgentNum);
savefile.collisionCBFvalue = zeros(1,AgentNum);


optresult = zeros(3,AgentNum);
detectNum = zeros(1,AgentNum);


Kd = 0;

while(~endflag)
    if t == 0
        input('PRESS ENTER to START')
        disp('START!!!!!!!!!')
        t_sim_start = tic;
        last_toc = toc(t_sim_start);
    end
    %%% real time execution
    samplingtime = toc(t_sim_start)-last_toc;
    T = samplingtime;
    last_toc = toc(t_sim_start);
    t = t+samplingtime;
    
    %%%%%%%%%%%%%%%% Get agent positions  %%%%%%%%%%%%%
    if real
        for i = 1:AgentNum
%          Receive PoseStamped message (sent from vrpn_client)
           pose_msg = mqttinterface.receive(sub_poses{i});
           x(i) = pose_msg.pose.position.x;
           y(i) = pose_msg.pose.position.y;
           z(i) = pose_msg.pose.position.z;
           orientation{i} = pose_msg.pose.orientation;
        end
    end
    
    %%%%%%%%%%%%%%%% JoyStick read %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    joy_msg = mqttinterface.receive(sub_joy{1});
    Lbutton = joy_msg.buttons(5);
    Rbutton = joy_msg.buttons(6);
    Perception(1,1:2) = [Lbutton Rbutton];


    endflag = joy_msg.buttons(7);
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%% detection %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    targetVector = zeros(2,AgentNum);
    if real
        for i = 1:AgentNum
            detection = mqttinterface.receive(sub_obj{i});
            detectNum(i) = size(detection.detections,1);
            if detectNum(i)
                for cnt=1:detectNum(i)
                    detections = detection.detections(cnt);
                    if ((detections.results.id==72)) && (detections.results.score > 0)
%                     if (detections.results.score > 0)
                        Perception(i) = 1;
                        centerOnImage = detections.bbox.center;
                        targetVector(:,i) = [1.0*(480/2-centerOnImage.y)/480; 1.8*(856/2-centerOnImage.x)/856];
                        %%%% bebop camera image size = 856*480 %%%%%%%%%%%
                        %%%  --> x          856
                        %%% | ---------------
                        %%% V |             |
                        %%% y |             |
                        %%%   --------------
                        %%% 480
                    end
                end
            end
        end
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    
%%%%%%%%%% update field weight%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Z = updateWeight(x,y,Z,Perception,samplingtime);
%%%%%%%%%% update energy data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for i=1:AgentNum
        E(i) = SimpleEnergyModel(E(i), Kd, charge_flag(i), samplingtime);
        Energy_msg.data = E(i);
        mqttinterface.send(pub_energy{i}, Energy_msg);
        chargeInfo(i) = getChargeCBF(x(i),y(i),E(i),Emin,Kd,k_charge,charge.pos(:,i),radius_charge);
    end
    
%%%%%%%%%% calculate voronoi region %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% and CBF for persistent coverage %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [u_nom(1,:),u_nom(2,:),Voronoi,persistCBF]=voronoi_ode_dis(x,y,Z,true);
    
    
%%%%%%%%%% get the target information from image or joy button %%%%%%%%%%    
    if real
        for i=1:AgentNum
            quat = [orientation{i}.w orientation{i}.x orientation{i}.y orientation{i}.z];
            rotm = quat2rotm(quat);
            rotTargetVector = rotm*[targetVector(:,i);0];
            pos = [x(i); y(i)] + rotTargetVector(1:2) - [0;0]; % direction to target(obtain from image)-offset to capture target
            theta = [0];
            norm = [2]; 
            width = [0.1;0.1 ];% target size
            targetInfo(i) = getPnomCBF(pos,theta,norm,width);
        end
    else
        for i=1:AgentNum
            pos = [x(i); y(i)] + targetVector(:,i) - [0.5; 0]; % direction to target(obtain from image)-offset to capture target
            theta = [0];
            norm = [2]; 
            width = [0.5;0.5 ];% target size
            targetInfo(i) = getPnomCBF(pos,theta,norm,width);
        end
    end
%%%%%%%%%% display the eval func for coverage %%%%%%%%%%%%%%%%%%%%%%%%%%%
    J = sum(getVoronoiEval(x,y,Voronoi.Region,Z))
% Perception
    
%%%%%%%%%% for collision avoidance %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for i=1:AgentNum
        x_others = x;%xyz
        y_others = y;
        x_others(i) = [];
        y_others(i) = [];
        collisionInfo(i) = getCollisionCBF(x(i),y(i),x_others,y_others,r);
    end
    
    
%%%%%%%%%% Plot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if matlab_plot
        voronoi_plot_dis(x,y,Voronoi,Z,fieldInfo,targetInfo,Perception,E)
    elseif ~real
        for i=1:AgentNum
            pose_msg.pose.position.x = x(i);
            pose_msg.pose.position.y = y(i);
            pose_msg.pose.position.z = 1;

            mqttinterface.send(pub_poses{i}, pose_msg);
        end
    end    
    if rviz_info_plot
        %%% Information Reliability msg send
        Z_ = flipud(Z); % to set upper is bigger y coordinate value.
        IR_msg.data = reshape(Z_', [1, num_grid_x*num_grid_y]);
        mqttinterface.send(info_topic, IR_msg);
    end

    
%%%%%%%%% Optimization %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for i=1:AgentNum
        optresult(:,i) = QP(x(i),y(i),u_nom(:,i),fieldInfo,chargeInfo(i),persistCBF(i),Perception(i),targetInfo(i),collisionInfo(i));
    end

%     optresult
%     CBF = f_con{1,1}{1,1}(x,y)
    u_opt = optresult(1:2,:);
    u_z = kz*(z_ref - z);
    
    
    

    
%%%%%%%%%% charging %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Within Charging Station and Energy Level is low, send land.
    for i=1:AgentNum
        if sqrt((x(i)-charge.pos(1,i))^2 + (y(i)-charge.pos(2,i))^2) <= radius_charge  &&  E(i) <= Emin + (Echarge - Emin)/3 && land_flags(i) == 0
            charge_flag(i) = 1;
            u_opt(:,i) = [0;0];            
            land_flags(i) = 1
         
%            Landing in charging stationland, not enough charged. Do nothing
        elseif land_flags(i) == 1 && E(i) <= Echarge
            u_opt(:,i) = [0;0];
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if real
                if bebop
                    takeoffmsg.data = '';
                    landmsg.data = 'land';
                    mqttinterface.send(pub_takeoffs{i}, takeoffmsg);
                    mqttinterface.send(pub_lands{i}, landmsg);   
                elseif crazyflie
                    mqttinterface.send(pub_lands{i}, landmsg);
                end
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
%         Landing in charging station, enough charged. Send takeoff.
        elseif land_flags(i) == 1 && E(i) >= Echarge
            charge_flag(i) = 0;           
            land_flags(i) = 0;
            
    
           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
           if real
%                 u_fullinCS([x(i);y(i)], drone_pub, quat, k);
                if bebop
                    takeoffmsg.data = 'takeoff';
                    landmsg.data = '';
                    mqttinterface.send(pub_takeoffs{i}, takeoffmsg);
                    mqttinterface.send(pub_lands{i}, landmsg);   
                elseif crazyflie
                    mqttinterface.send(pub_takeoffs{i}, takeoffmsg);
                end
           end
           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % Out of Charging Station OR within charging station with enough energy, send velocity input.
        else
            if real
                quat = [orientation{i}.w orientation{i}.x orientation{i}.y orientation{i}.z];
                rotm = quat2rotm(quat);
                body_vel = rotm' * [u_opt(:,i); 0];
                u_opt(:,i) = body_vel(1:2);
                if z(i) > z_thresh
                        vel_msg.linear.x = u_opt(1,i);
                        vel_msg.linear.y = u_opt(2,i);
                        vel_msg.angular.z = -quat(4);
                        if abs(z(i) - z_ref) < 0.1
                            vel_msg.linear.z = 0;
                        else
                            vel_msg.linear.z = u_z(i);
                        end
                        if bebop
                            takeoffmsg.data = '';
                            landmsg.data = '';
                            mqttinterface.send(pub_takeoffs{i}, takeoffmsg);
                            mqttinterface.send(pub_lands{i}, landmsg);
                        end
                        mqttinterface.send(pub_vels{i}, vel_msg);
                end
            end
        end
    end

%%%%%%%%%%%  position update for simulation %%%%%%%%%%%%%
    if ~real
        u_opt(abs(u_opt.*samplingtime)>0.4) = 0;
        x = x+(u_opt(1,:))*samplingtime;
        y = y+(u_opt(2,:))*samplingtime;
        z = z + u_z * samplingtime;
    end

    
    
    
    %%%%% for save %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for i=1:AgentNum
        hxf(1,i) = fieldInfo.hx(x(i),y(i));
        hxt(1,i) = targetInfo(i).hx(x(i),y(i))*Perception(i);
        hxc(1,i) = chargeInfo(i).hx;
        hxp(1,i) = persistCBF(i).hx;
        if AgentNum ~= 1
            hxc_(1,i) = min(collisionInfo.hx);
        end
    end
    savefile.fieldCBFvalue = [savefile.fieldCBFvalue;hxf];
    savefile.chargeCBFvalue = [savefile.chargeCBFvalue;hxc];
    savefile.persistentCBFvalue = [savefile.persistentCBFvalue;hxp];
    savefile.targetCBFvalue = [savefile.targetCBFvalue; hxt];
    savefile.time = [savefile.time;t];
    savefile.energy = [savefile.energy;E];
    savefile.J = [savefile.J; getVoronoiEval(x,y,Voronoi.Region,Z)'];
    savefile.uX = [savefile.uX; optresult(1,:)];
    savefile.uY = [savefile.uY; optresult(2,:)];
    savefile.w = [savefile.w; optresult(3,:)];
    savefile.x = [savefile.x; x];
    savefile.y = [savefile.y; y];
    savefile.collisionCBFvalue = [savefile.collisionCBFvalue;hxc_];
    
end

%% End of experiment
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if real
    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    
    for i=1:AgentNum
        if bebop
            takeoffmsg.data = '';
            landmsg.data = '';
            mqttinterface.send(pub_takeoffs{i}, takeoffmsg);
            mqttinterface.send(pub_lands{i}, landmsg);
        end
        mqttinterface.send(pub_vels{i},vel_msg)
    end

end
disp('END!!!!!!!!!!!!')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Plot figure

%%%% J value plot
figure
sumJ = sum(savefile.J,2);
plot(savefile.time,sumJ)
hold on
grid on
plot(savefile.time,goalJ*ones(size(savefile.time)))

%%
%%%% charging plot
figure
for i=1:AgentNum
    plot(savefile.time,savefile.energy(:,i))
    hold on
    grid on
end
plot(savefile.time,Emin*ones(size(savefile.time)))
%%%% 
%%

%%%% w value plot
figure
for i=1:AgentNum
    plot(savefile.time,savefile.w(:,i))
    hold on
    grid on
end
%%
figure
for i=1:AgentNum
    plot(savefile.time,savefile.collisionCBFvalue(:,i))
    hold on
    grid on
end
