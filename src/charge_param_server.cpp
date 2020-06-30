#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <task_switch/charge_paramsConfig.h>

void callback(task_switch::charge_paramsConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Update");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "charge_parameter");

  dynamic_reconfigure::Server<task_switch::charge_paramsConfig> server;
  dynamic_reconfigure::Server<task_switch::charge_paramsConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}
