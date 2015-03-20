//NODE to dynamically update a cropbox filter's params
//NODE to MOVE YOUBOT AROUND AN OBJECT using PID.
//to do: Kp,Ki,Kd dynamic reconfigure
//to do: Distance from object - dynamic reconfigure
#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"
#include <dynamic_reconfigure/server.h>
#include <youbot_499/youbot_circle_pidConfig.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_listener.h>

const double PI = 3.14159265358979323846;

ros::Publisher pub;
tf::TransformListener* listener;
tf::Transform g_A5ToAsus;
tf::Transform g_AsusToA5;

volatile float distance = 1;

void callback1(youbot_499::youbot_circle_pidConfig &config, uint32_t level)
{
  distance = config.refd;
}

void initialize()
{
  
	tf::Matrix3x3 rot;
	tf::Vector3 t;
	
	// --- arm_link_5 to ASUS center --- //
	
	rot.setValue(0.0, -1.0, 0.0,
				 1.0, 0.0, 0.0,
				 0.0, 0.0, 1.0);
	t.setValue(0.088, 0.0, 0.22);
		
	g_A5ToAsus.setBasis(rot);
	g_A5ToAsus.setOrigin(t);


	// --- ASUS center to arm_link_5 --- //
		
	g_AsusToA5 = g_A5ToAsus.inverse();
}

void object_asus()
{
  geometry_msgs::Point P;
  tf::StampedTransform g_A5ToLaser;
  tf::Transform g_ObjToAsus;
  tf::StampedTransform g_ObjToLaser;
  try {
    listener->lookupTransform("/arm_link_5","/base_laser_front_link",ros::Time(0),g_A5ToLaser);
    ROS_INFO("Running....");
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  
  tf::Vector3 t( distance, 0, 0 );
  g_ObjToLaser.setOrigin( t );
  g_ObjToLaser.setBasis( tf::Matrix3x3::getIdentity() );
  g_ObjToAsus = g_AsusToA5 * g_A5ToLaser * g_ObjToLaser;
  tf::Vector3 final = g_ObjToAsus.getOrigin();
  P.x = final.getX();
  P.y = final.getY();
  P.z = final.getZ();
  pub.publish(P);
  //ROS_INFO("Running....");
 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cropbox_dynamic");
  ros::NodeHandle n;
  
  listener = new tf::TransformListener();
  pub = n.advertise<geometry_msgs::Point>("obj_Asus", 1000);
  initialize();
   while (1){
  dynamic_reconfigure::Server<youbot_499::youbot_circle_pidConfig> pr_srv;
  dynamic_reconfigure::Server<youbot_499::youbot_circle_pidConfig>::CallbackType cb1;
  cb1 = boost::bind(&callback1, _1, _2);
  pr_srv.setCallback(cb1);
  ros::Duration(1.0).sleep();
  object_asus();
  }
  ros::spin();
  return 0;
}
