/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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
 *********************************************************************/

//! \author Vijay Pradeep

#include <cstdio>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <unistd.h>
#include <termios.h>

int main(int argc, char** argv)
{
  // Setup terminal settings for getchar
  const int fd = fileno(stdin);
  termios prev_flags ;
  tcgetattr(fd, &prev_flags) ;
  termios flags ;
  tcgetattr(fd,&flags);
  flags.c_lflag &= ~ICANON;  // set raw (unset canonical modes)
  flags.c_cc[VMIN]  = 0;     // i.e. min 1 char for blocking, 0 chars for non-blocking
  flags.c_cc[VTIME] = 0;     // block if waiting for char
  tcsetattr(fd,TCSANOW,&flags);

  ros::init(argc, argv, "keyboard_float_generator");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<std_msgs::Float64>("keyboard_float", 10);

  const double increment = .01;

  ROS_INFO("Press '=' to increment, and '-' to decrement");

  std_msgs::Float64 shift_val;
  shift_val.data = 0.0;
  while (nh.ok())
  {
    char c = getchar();

    bool should_publish = false;


    switch (c)
    {
      case '=': shift_val.data += increment;  should_publish = true; break;
      case '+': shift_val.data += increment;  should_publish = true; break;
      case '-': shift_val.data -= increment;  should_publish = true; break;
      default:
        usleep(100);
        break;
    }

    if (should_publish)
    {
      ROS_INFO("Publishing Float: % .3f", shift_val.data);
      pub.publish(shift_val);
    }
  }

  tcsetattr(fd,TCSANOW, &prev_flags) ;         // Undo any terminal changes that we made

  return 0;
}
