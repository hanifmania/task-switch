#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>

// This node looks transform named 'base_link' relative to 'world'
// and publishes PoseStamped msg to 'pose' topic.

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf2pose");

    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);

    tf::TransformListener listener;

    ros::Rate rate(40.0);
    while (nh.ok())
    {
        geometry_msgs::PoseStamped baselink_pose;
        ros::Time now = ros::Time::now();
        std::string world_frame = "world";
        baselink_pose.header.stamp = now;
        baselink_pose.header.frame_id = world_frame;

        tf::StampedTransform transform;
        try
        {
            listener.lookupTransform("world", "base_link", ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        baselink_pose.pose.position.x = transform.getOrigin().x();
        baselink_pose.pose.position.y = transform.getOrigin().y();
        baselink_pose.pose.position.z = transform.getOrigin().z();
        baselink_pose.pose.orientation.x = transform.getRotation().x();
        baselink_pose.pose.orientation.y = transform.getRotation().y();
        baselink_pose.pose.orientation.z = transform.getRotation().z();
        baselink_pose.pose.orientation.w = transform.getRotation().w();

        pub.publish(baselink_pose);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
};
