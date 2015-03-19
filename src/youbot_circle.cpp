//NODE to MOVE YOUBOT AROUND AN OBJECT using PID.
//to do: Kp,Ki,Kd dynamic reconfigure
//to do: Distance from object - dynamic reconfigure
#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <dynamic_reconfigure/server.h>
#include <youbot_499/youbot_circle_pidConfig.h>

volatile float Ki,Kp,Kd;
volatile float ref_distance;

ros::Publisher pub;

void callback1(youbot_499::youbot_circle_pidConfig &config, uint32_t level)
{
  ROS_INFO("Dynamically reconfigure : \nDistance to object = %f Kp = %f Ki= %f Kd = %f",
           config.refd,
           config.kp,
           config.ki,
	   config.kd);
  ref_distance = config.refd;
  Kp=config.kp;
  Ki=config.ki;
  Kd=config.kd;
}

int minimum_vector(std::vector<float> a){  //function to calculate the minimum of a vector
  int c=10000;
  int index;
  for (int i=0;i<a.size();i++)
    {
      if (c>a[i])
	{
	  c=a[i];
	  index=i;
	}
    }
  std::cout<<a[index]<<" is the minimum range\n"<<std::endl;
  return index;
}


void distance_controller(float current_distance)
{
  geometry_msgs::Twist command;
  command.linear.x=pid(ref_distance,current_distance);
  pub.publish(command);
}

void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan){
     //scan->ranges[] are laser readings
  std::cout<<scan->ranges.size()<<" type of range values\n"<<std::endl;
  int index = minimum_vector(scan->ranges);
  distance_controller(scan->ranges[index]);  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "youbot_circle");
  ros::NodeHandle n;
  
  pub = n.advertise<geometry_msgs::Twist> ("cmd_vel", 1);
  ros::Subscriber scanSub=n.subscribe<sensor_msgs::LaserScan>("base_scan",10,&processLaserScan);
  dynamic_reconfigure::Server<youbot_499::youbot_circle_pidConfig> pr_srv;
  dynamic_reconfigure::Server<youbot_499::youbot_circle_pidConfig>::CallbackType cb1;
  cb1 = boost::bind(&callback1, _1, _2);
  pr_srv.setCallback(cb1);
  ROS_INFO("Running....");
  ros::spin();
  return 0;
}
