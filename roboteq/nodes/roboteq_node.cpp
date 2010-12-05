/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Arizona State University.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Srikanth Saripalli on 12/04/10
*********************************************************************/

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <time.h>
#include <sstream>


#include "roboteq/RoboteqDevice.h"
#include "roboteq/ErrorCodes.h"
#include "roboteq/Constants.h"
#include "roboteq/Raven.h"


#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

using namespace std;


void publish_pose(tf::TransformBroadcaster *odom_broadcaster, ros::Publisher *odom_pub)
{
  ROS_INFO("Timer trigerred at 10Hz");
  nav_msgs::Odometry odom_pose;
  odom_pub->publish(odom_pose);
}

void cmdvelcallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("cmd vel callback");
}

int main( int argc, char **argv )
{
    ros::init( argc, argv, "roboteq_node" );
    ros::NodeHandle n;
    tf::TransformBroadcaster odom_broadcaster;
    // no delay: we always want the most recent data
    ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);
	
    //Params
    std::string portname;
    int baudrate;
    std::string frame_id;
    n.param<std::string>("port", portname, "/dev/ttyUSB0");
    n.param("baudrate", baudrate, 115200);
    n.param<std::string>("frame_id", frame_id, "/base_frame");
   
    RoboteqDevice device;
     
    int status = device.Connect(portname);

    if(status != RQ_SUCCESS){
      cout << "Error connecting to the device: "<< status <<"."<< endl;
      return -1;
    }
    ROS_INFO("Roboteq -- Successfully connected to the Roboteq HDC2450");
    ros::Duration(0.05).sleep(); //sleep for 50 ms 
    
    	
    ROS_INFO("- SetConfig(_DINA, 1, 1)...");
    if((status = device.SetConfig(_DINA, 1, 1)) != RQ_SUCCESS)
      cout<<"failed --> "<<status<<endl;
    else
      ROS_INFO("succeeded.");
    ros::Duration(0.05).sleep(); //sleep for 50 ms 

    int result;
    ROS_INFO("- GetConfig(_DINA, 1)...");
    if((status = device.GetConfig(_DINA, 1, result)) != RQ_SUCCESS)
      cout<<"failed --> "<<status<<endl;
    else
      cout<<"returned --> "<<result<<endl;
    ROS_INFO("Roboteq -- Successfull setup of the Roboteq HDC2450");
    ros::Duration(0.05).sleep(); //sleep for 50 ms 

    ROS_INFO("- GetValue(_ANAIN, 1)...");
    if((status = device.GetValue(_ANAIN, 1, result)) != RQ_SUCCESS)
      cout<<"failed --> "<<status<<endl;
    else
      cout<<"returned --> "<<result<<endl;
    ros::Duration(0.05).sleep(); //sleep for 50 ms 

    ROS_INFO("- SetCommand(_GO, 1, 1)...");
    if((status = device.SetCommand(_GO, 1, 1)) != RQ_SUCCESS)
      cout<<"failed --> "<<status<<endl;
    else
      cout<<"succeeded."<<endl;
    ros::Duration(0.05).sleep(); //sleep for 50 ms 

    //subscribe to the motor_control topic to listen to motor commands
    ros::Subscriber roboteq_sub = n.subscribe("cmd_vel", 1, cmdvelcallback, noDelay);

    //setup publisher data for publishing odometry data
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom",1);

    //setup publisher for publishing motor status related data


    ros::Rate r(20);
    while(ros::ok()){
        publish_pose(&odom_broadcaster, &odom_pub);
    	ros::spinOnce();
	r.sleep();
    }

    ROS_INFO("Roboteq: exiting main loop");
    device.Disconnect();
    return 0;
}

