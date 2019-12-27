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

if ~matlab_plot
    pub_plot = "plot_data";
    mqttinterface.add_publisher(pub_plot);
end


%%%center reference 
ref_plot = "reference";
mqttinterface.add_publisher(ref_plot);




mqttinterface.connect();
