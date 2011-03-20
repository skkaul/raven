#include "roboteq/Raven.h"
#include <ros/ros.h>  
#include"roboteq/config.h"
#include "roboteq/RoboteqDevice.h"
#include "roboteq/ErrorCodes.h"
#include "roboteq/Constants.h"
#include "roboteq/Raven.h"

namespace{
roboteq::config c;
 RoboteqDevice r;
int left_motor_amps;
    int right_motor_amps;
    int left_battery_amps;
    int right_battery_amps;
}
void get_config(ros::Publisher *pub_config)
{
if(r.GetValue(_A, 1, left_motor_amps)!=RQ_SUCCESS)
  ROS_INFO("config data motor1  decoding failed"); 
  if(r.GetValue(_A, 2, right_motor_amps)!=RQ_SUCCESS)
ROS_INFO("config data decoding failed"); 
  c.MotorAmps1=left_motor_amps;
  c.MotorAmps2=right_motor_amps;
if(r.GetValue(_BA, 1, left_battery_amps)!=RQ_SUCCESS)
 ROS_INFO("config data decoding failed");
if(r.GetValue(_BA, 2, right_battery_amps)!=RQ_SUCCESS)
 ROS_INFO("config data decoding failed");
 
c.BatteryAmps1=left_battery_amps;
 c.BatteryAmps2=right_battery_amps;
 c.header.stamp=ros::Time::now();
 //c.header.frameid="config";
 pub_config->publish(c);
}
int main( int argc, char **argv )
{
    ros::init( argc, argv, "roboteq_config" );
    ros::NodeHandle n;
    ros::Publisher pub_config= n.advertise<roboteq::config>("config",5);
   
    ros::Rate x(20);
     while(ros::ok()){
        get_config(&pub_config);
    	ros::spinOnce();
	x.sleep();
    }
 
}


