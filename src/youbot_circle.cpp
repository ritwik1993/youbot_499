//NODE to MOVE YOUBOT AROUND AN OBJECT using PID.
#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <dynamic_reconfigure/server.h>
#include <youbot_499/youbot_circle_pidConfig.h>

volatile float Ki,Kp,Kd; //Distance PID Gains
volatile float Kii,Kpp,Kdd; //Orientation PID gains
volatile float ref_distance;
volatile float eint=0;
volatile float eprev=0;
volatile float e1int=0;
volatile float e1prev=0;
volatile float estop=1;

ros::Publisher pub;
geometry_msgs::Twist command;

void callback1(youbot_499::youbot_circle_pidConfig &config, uint32_t level)
{
  ROS_INFO("Dynamically reconfigure : \nEmergency Stop : %i\nLinear Y velocity = %f Distance to object = %f\nDistance Kp = %f Distance Ki= %f Distance Kd = %f\nOrientation Kp = %f Orientation Ki = %f Orientation Kd = %f",
	   (int)config.estop,
	   config.y,
           config.refd,
           config.kp,
           config.ki,
	   config.kd,
	   config.kpp,
           config.kii,
	   config.kdd);
  estop=config.estop;
  command.linear.y=config.y;
  ref_distance = config.refd;
  Kp=config.kp;
  Ki=config.ki;
  Kd=config.kd;
  Kpp=config.kpp;
  Kii=config.kii;
  Kdd=config.kdd;
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
  return index;
}

float pid_distance(float ref,float sensor){
  float e,u;
  float edot;
  float EINTMAX=1.0;
  e=ref-sensor;
  edot=e-eprev;
  eint=eint+e;
  if (eint > EINTMAX) eint = EINTMAX;
  if (eint < -EINTMAX) eint = -EINTMAX;
  u=Kp*e+Ki*eint+Kd*edot;
  eprev=e;
  if (u>0.5)u=0.5;
  if (u<-0.5)u=-0.5;  //saturate control
  return u;
}

float pid_orientation(float ref,float sensor){
  float e,u;
  float edot;
  float EINTMAX=1.0;
  e=ref-sensor;
  edot=e-e1prev;
  e1int=e1int+e;
  if (e1int > EINTMAX) e1int = EINTMAX;
  if (e1int < -EINTMAX) e1int = -EINTMAX;
  u=Kpp*e+Kii*e1int+Kdd*edot;
  e1prev=e;
  if (u>0.5)u=0.5;
  if (u<-0.5)u=-0.5; //saturate control
  return u;
}


void distance_controller(float current_distance)
{
  command.linear.x=pid_distance(ref_distance,current_distance);
  std::cout<<current_distance<<" is the current distance\n"<<std::endl;
}

void orientation_controller(float angle)
{
  command.angular.z=pid_orientation(0,angle);
  std::cout<<angle<<" is the current angle\n"<<std::endl;
}

float angle_from_index(int index,float min,float max,float inc){
  float angle =  inc*index + min;
  return angle;
}

void controller(float current_distance,float angle){
  if (estop == 1)
    {
      command.linear.x=0;
      command.linear.y=0;
      command.linear.z=0;
      command.angular.x=0;
      command.angular.y=0;
      command.angular.z=0;
    }
  else
    {
      distance_controller(current_distance);
      orientation_controller(angle);
    }
  pub.publish(command);
}

void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan){
     //scan->ranges[] are laser readings
  int index = minimum_vector(scan->ranges);
  float angle=angle_from_index(index,scan->angle_min,scan->angle_max,scan->angle_increment);
  controller(scan->ranges[index],angle);  
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
