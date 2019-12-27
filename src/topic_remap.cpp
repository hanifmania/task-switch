#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

void cmd_velCallback(const geometry_msgs::Twist::ConstPtr &msg);

void cmd_velCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "topic_remap");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("twist", 1);
    ros::Subscriber sub = nh.subscribe("cmd_vel2", 1, cmd_velCallback);
    ros::spin();
    return 0;
}