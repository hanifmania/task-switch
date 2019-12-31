%% MQTT setting

mqttinterface = MqttInterface(['matlab_mqtt_node_',num2str(datenum(datetime('now')))], '127.0.0.1', 1883);

joy_topic = 'joy';
sub_joy = {};
sub_joy{1} = joy_topic;
mqttinterface.add_subscriber(sub_joy{1});


%%% settings for topic name to subscribe and publish %%%%%%%%
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




%%% registration for subscribe or publish%%%%%%%%%%%%%%%%%%%%%%%%%





%%%%%%% real experiment %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% detection topic 
%%%% ros: /bebop10x/objects -> mqtt: objects0x
%%%% 
%%%% /bebop10x/objects: vision_msgs/Detection2DArray
%%%% |
%%%% |--detections: vision_msgs/Detection2D[]
%%%%   |
%%%%   |---[0]: vision_msgs/Detection2D
%%%%   |---[1]
%%%%   |---[2]
%%%%   |  |---bbox: vision_msgs/BoundingBox2D
%%%%      |  |---center: geometry_msgs/Pose2D
%%%%      |  |  |-theta
%%%%      |  |  |-x
%%%%      |  |  |-y
%%%%      |  |---size_x
%%%%      |  |---size_y
%%%%      |---header
%%%%      |---results
%%%%      |  |---[0] :vision_msgs/ObjectHypothesisWithPose
%%%%      |     |---id : int64
%%%%      |     |---pose
%%%%      |     |---score
%%%%      ----source_img
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





%%%%%%publish velocity,takeoff,land and subscribe poses
if real    
    pub_vels = {};
    sub_poses = {};
    pub_takeoffs = {};
    pub_lands = {};

% Create str of topic names
    for i=1:AgentNum
        agent_num_str = num2str(drone_list(i));
        pub_vels{i} = strcat(vel_topic, agent_num_str);
        sub_poses{i} = strcat(pose_topic, agent_num_str);
        pub_takeoffs{i} = strcat(takeoff_topic, agent_num_str);
        pub_lands{i} = strcat(land_topic, agent_num_str);
        sub_obj{i} = strcat(obj_topic, agent_num_str);
    end    
    
%     Add subscribers and publisher topics to MQTT
    for i=1:AgentNum
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
   
%%%%%%%% simulation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% If it's simulation, publish poses
else
    pub_poses = {};
    for i=1:AgentNum
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


%%%%%%% for plot and visualize%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if ~matlab_plot
    pub_plot = "plot_data";
    %%% information map topic
    info_topic = 'InformationReliability';

    
    mqttinterface.add_publisher(pub_plot);
    mqttinterface.add_publisher(info_topic);
    
    
    % Set Initial message for Information Reliability.
    IR_msg.layout.dim(1).label = "y";
    IR_msg.layout.dim(1).size = mesh_acc(2);
    IR_msg.layout.dim(1).stride = mesh_acc(2) * mesh_acc(1);
    IR_msg.layout.dim(2).label = "x";
    IR_msg.layout.dim(2).size = mesh_acc(1);
    IR_msg.layout.dim(2).stride = mesh_acc(1);
    IR_msg.data = reshape(Z',1,{});
end


%%% energy plot
pub_energy = {};

energy_topic = 'Energy';
for i=1:AgentNum
    agent_num_str = num2str(drone_list(i));
    pub_energy{i} = strcat(energy_topic, agent_num_str);
    mqttinterface.add_publisher(pub_energy{i});
end




mqttinterface.connect();
