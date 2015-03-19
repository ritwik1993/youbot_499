//NODE TO CONTROL VEL THROUGH DYNAMIC RECONFIGURE
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <dynamic_reconfigure/server.h>
#include <youbot_499/cmd_veldynamicConfig.h>

ros::Publisher pub;

void callback(youbot_499::cmd_veldynamicConfig &config, uint32_t level)
{
  ROS_INFO("Dynamically reconfigure : \nlinear.x = %f linear.y = %f linear.z= %f\nangular.x = %f angular.y = %f angular.z = %f",
           config.x,
           config.y,
           config.z,
	   config.x1,
	   config.y1,
	   config.z1);
  geometry_msgs::Twist t;
  t.linear.x=config.x;
  t.linear.y=config.y;
  t.linear.z=config.z;
  t.angular.x=config.x1;
  t.angular.y=config.y1;
  t.angular.z=config.z1;
  pub.publish(t);
  // do nothing for now
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "youbot_vel_dynamic");
  ros::NodeHandle n;
  //ros::Publisher
  pub = n.advertise<geometry_msgs::Twist> ("cmd_vel", 1);
  dynamic_reconfigure::Server<youbot_499::cmd_veldynamicConfig> dr_srv;
  dynamic_reconfigure::Server<youbot_499::cmd_veldynamicConfig>::CallbackType cb;
  cb = boost::bind(&callback, _1, _2);
  dr_srv.setCallback(cb);
  ROS_INFO("Running....");
  ros::spin();
  return 0;
}
