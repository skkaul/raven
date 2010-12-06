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
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>




using namespace std;

//globals
namespace {

    RoboteqDevice device;
    ros::Time current_time, last_time;
    double posx = 0.0;
    double posy = 0.0;
    double posth = 0.0;
};


void publish_pose(tf::TransformBroadcaster *odom_broadcaster, ros::Publisher *odom_pub)
{

  double vx, vy, vth;
  double delta_x, delta_y, delta_th;

  //get encoder data from the roboteq 
  int left_enc_data, right_enc_data;
  float wheel_circumference = WHEEL_DIAMETER * M_PI;
  float left_enc = left_enc_data * wheel_circumference/ENCODER_RESOLUTION;
  float right_enc = right_enc_data * wheel_circumference/ENCODER_RESOLUTION;
  float diff_enc = (right_enc - left_enc)/WHEEL_BASE_WIDTH;
  
  float dist = (left_enc + right_enc)/2.0;
  delta_th = (double) diff_enc;
  delta_x = cos(posth) * dist; //check
  delta_y = -sin(posth) * dist; //check

  current_time = ros::Time::now();

  //compute odometry in a typical way given the velocities of the robot
  double dt = (current_time - last_time).toSec();
  vx = delta_x/dt;
  vy = delta_y/dt;
  vth = delta_th/dt;

  posx += delta_x;
  posy += delta_y;
  posth += delta_th;
   
  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(posth);

  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = posx;
  odom_trans.transform.translation.y = posy;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  //send the transform
  odom_broadcaster->sendTransform(odom_trans);

  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.x = posx;
  odom.pose.pose.position.y = posy;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vth;
  
  odom_pub->publish(odom);
  last_time = current_time;
}

void cmdvelcallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
  float v_mag, mag_r;
  float vel_x, vel_y, vel_yaw;
  float  delta_v, v_left, v_right;
  float speed_left, speed_right;
  int left_cmd, right_cmd;
  
  ROS_INFO("cmd vel callback");

  vel_x = cmd_vel->linear.x;
  vel_y = cmd_vel->linear.y;
  vel_yaw = cmd_vel->angular.z;

  //Calculate magnitutde of the velocity vmag and Radius of Rotation mag_r
  v_mag = sqrt((pow(vel_x, 2)) + pow(vel_y, 2));
  if (vel_yaw == 0){
    mag_r = 0;
  }
  else {
    mag_r = v_mag/vel_yaw;
  }
        
  //Calculate the difference in velocity between the wheels
  if (mag_r == 0){
    delta_v = 0;
  }
  else {
    delta_v = 2.0 * ((v_mag * WHEEL_BASE_LENGTH)/mag_r);
  }
        
  //Calculate the velocity of each wheel
  if (v_mag == 0){
    v_left = vel_yaw * -1;
    v_right = vel_yaw;
  }else {
    v_left  = (v_mag - (0.5 * delta_v));
    v_right = (v_mag + (0.5 * delta_v));
  }

  //Calculate percentage of max velocity for each wheel
  if (v_left == 0){
    speed_left = 0;
  } else if ( v_left > MAX_WHEEL_VELOCITY) {
    speed_left = 1;
  } else if ( v_left < -MAX_WHEEL_VELOCITY) {
    speed_left = -1;
  } else {
    speed_left = v_left/MAX_WHEEL_VELOCITY;
  }

  if (v_right == 0){
    speed_right = 0;
  } else if ( v_right > MAX_WHEEL_VELOCITY) {
    speed_right = 1;
  } else if ( v_right < -MAX_WHEEL_VELOCITY) {
    speed_right = -1;
  } else {
    speed_right = v_right/MAX_WHEEL_VELOCITY;
  }

  left_cmd  = int(speed_left * MOTOR_RANGE);
  right_cmd = int(speed_right * MOTOR_RANGE);

  //send these to the roboteq controller
  // TO DO
  int status;
  ROS_INFO("- SetCommand(_GO, 1, 1)...");
  if((status = device.SetCommand(_GO, 1, left_cmd)) != RQ_SUCCESS)
    cout<<"failed --> "<<status<<endl;
  //sleep for 10 ms do we have to do this 
  ros::Duration(0.01).sleep(); //sleep for 10 ms 
  if((status = device.SetCommand(_GO, 2, right_cmd)) != RQ_SUCCESS)
    cout<<"failed --> "<<status<<endl;

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
    n.param<std::string>("port", portname, portname);
    n.param("baudrate", baudrate, baudrate);
   
     
    int status = device.Connect(portname);
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    if(status != RQ_SUCCESS){
      cout << "Error connecting to the device: "<< status <<"."<< endl;
      return -1;
    }
    ROS_INFO("Roboteq -- Successfully connected to the Roboteq HDC2450");
    ros::Duration(0.01).sleep(); //sleep for 10 ms 
    
    	
    ROS_INFO("- SetConfig(_DINA, 1, 1)...");
    if((status = device.SetConfig(_DINA, 1, 1)) != RQ_SUCCESS)
      cout<<"failed --> "<<status<<endl;
    else
      ROS_INFO("succeeded.");
    ros::Duration(0.01).sleep(); //sleep for 10 ms 

    int result;
    ROS_INFO("- GetConfig(_DINA, 1)...");
    if((status = device.GetConfig(_DINA, 1, result)) != RQ_SUCCESS)
      cout<<"failed --> "<<status<<endl;
    else
      cout<<"returned --> "<<result<<endl;
    ROS_INFO("Roboteq -- Successfull setup of the Roboteq HDC2450");
    ros::Duration(0.01).sleep(); //sleep for 10 ms 

    //subscribe to the motor_control topic to listen to motor commands
    ros::Subscriber roboteq_sub = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1, cmdvelcallback, noDelay);

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

