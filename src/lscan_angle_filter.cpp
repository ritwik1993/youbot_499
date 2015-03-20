//NODE to filter the laser scanner data based on dynamically reconfigured min and max angle
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <dynamic_reconfigure/server.h>
#include <youbot_499/lscan_angle_filterConfig.h>


static float pi = 3.141592;
ros::Publisher pub;
sensor_msgs::LaserScan filtered_scan;
volatile float mina=-1.5;   //Default values in radians
volatile float maxa=+1.5;   //Default values in radians

void callback1(youbot_499::lscan_angle_filterConfig &config, uint32_t level)
{
  ROS_INFO("Dynamically reconfigure : \nMin Angle : %f \nMax Angle = %f\n",
	   config.minang,
	   config.maxang);
  mina=(config.minang*pi)/180;
  maxa=(config.maxang*pi)/180;
}

float angle_from_index(int index,float min,float max,float inc){
  float angle =  inc*index + min;
  return angle;
}

int index_from_angle(float angle,float min,float max,float inc){   //Precision not required as we only use this for filtering
  float index =  angle - min;
  index=index/inc;
  return (int)index;
}


void processLaserScan(sensor_msgs::LaserScan scan){
     //scan->ranges[] are laser readings
  sensor_msgs::LaserScan filtered_scan = scan;
  
  int min_index=index_from_angle(mina,scan.angle_min,scan.angle_max,scan.angle_increment);
  int max_index=index_from_angle(maxa,scan.angle_min,scan.angle_max,scan.angle_increment);
  for (int i=0;i<scan.ranges.size();i++)
    {
      if (i<min_index)filtered_scan.ranges[i]=1000;  //make it absurdly high
      if (i>max_index)filtered_scan.ranges[i]=1000;
    }
  pub.publish(filtered_scan);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lscan_angle_filter");
  ros::NodeHandle n;
  
  pub = n.advertise<sensor_msgs::LaserScan> ("filtered_scan", 10);
  ros::Subscriber scanSub=n.subscribe<sensor_msgs::LaserScan>("base_scan",10,&processLaserScan);
  dynamic_reconfigure::Server<youbot_499::lscan_angle_filterConfig> pr_srv;
  dynamic_reconfigure::Server<youbot_499::lscan_angle_filterConfig>::CallbackType cb1;
  cb1 = boost::bind(&callback1, _1, _2);
  pr_srv.setCallback(cb1);
  ROS_INFO("Running....");
  ros::spin();
  return 0;
}
