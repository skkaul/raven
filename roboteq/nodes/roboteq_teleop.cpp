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


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <joy/Joy.h>
using namespace std;
namespace {

  int linear_,linear1_, angular_,angular1_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub;
  ros::Subscriber joy_sub;
};

void joycallback(const joy::Joy::ConstPtr& joy)
{
 
  ROS_INFO("cmd vel published");
  geometry_msgs::Twist cmd_vel;
  cmd_vel.angular.z=0;
    cmd_vel.linear.x=0;
/*
     do 
     { 
    if(joy->buttons[0])
      cmd_vel.angular.z = a_scale_*joy->buttons[angular_];//break;}//changed
else
  if(joy->buttons[2])
    cmd_vel.angular.z=-(a_scale_)*joy->buttons[angular1_];//break;}
  else
    if(joy->buttons[3])
      cmd_vel.linear.x = l_scale_*joy->buttons[linear_];//break;}//changed
    else
      if( joy->buttons[1] ) 
 
   cmd_vel.linear.x=-(l_scale_)*joy->buttons[linear1_];//break;
     
else
   vel_pub.publish(cmd_vel);
	}while((joy->buttons[0]==1)|(joy->buttons[1]==1)|(joy->buttons[2]==1)|(joy->buttons[3]==1));     
    vel_pub.publish(cmd_vel);
*/ 
if(joy->buttons[deadswitch_])
  {
    cmd_vel.angular.z=a_scale_*joy->axes[angular_];
  cmd_vel.linear.x=l_scale_*joy->axes[linear_];
  }
 else
   {
     cmd_vel.angular.z=0;
     cmd_vel.linear.x=0;
   }
  
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "roboteq_teleop");
  // no delay: we always want the most recent data
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);
 int switch1=5;
  int def=2;//(changed)
  // int def0=1;
  int def1=1;
  //int def2=2;
  double def5=1;
double def2=0.8

  //  double f1=4.8;//changing for debugging
  // int def2=0;

  ros::NodeHandle n;
  n.param("axis_linear", linear_, def1 );
  //n.param("axis_linear",linear1_,def0);
  n.param("axis_angular", angular_, def);
  // n.param("axis_angular",angular1_,def2);
  n.param("scale_angular", a_scale_, def5);
  n.param("scale_linear", l_scale_, def2);
  n.param("dead_switch",deadswitch_,switch1);


  vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  joy_sub = n.subscribe<joy::Joy>("joy",1,joycallback,noDelay);
  
  ros::Rate r(10);

   while(ros::ok()){
        vel_pub.publish(cmd_vel);
        ros::spinOnce();

        }

  return 0;
}


