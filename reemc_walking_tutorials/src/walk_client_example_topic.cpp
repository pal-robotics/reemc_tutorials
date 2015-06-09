/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2014, PAL Robotics, S.L.
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
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
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
 */

/*
 *  Created on: 1/11/2014
 *      Author: Luca Marchionni
 */


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

const std::string WALK_CMD_VEL_TOPIC = "/joy_vel";

int main(int argc, char **argv)
{

  ros::init(argc, argv, "walking_client_topic");
  ros::NodeHandle n;

  ros::Publisher cmd_publisher = n.advertise<geometry_msgs::Twist>(WALK_CMD_VEL_TOPIC,1 );
  sleep(5);


  /// Sending a positive linear speed cmd for performing a forward step
  geometry_msgs::Twist cmd_vel_forward;
  cmd_vel_forward.linear.x = 0.5;
  cmd_publisher.publish(cmd_vel_forward);
  sleep(3.0);

  geometry_msgs::Twist cmd_vel_backward;
  cmd_vel_backward.linear.x = -0.5;
  cmd_publisher.publish(cmd_vel_backward);
  sleep(3.0);

  geometry_msgs::Twist cmd_vel_left;
  cmd_vel_left.linear.y = 0.3;
  cmd_publisher.publish(cmd_vel_left);
  sleep(3.0);

  geometry_msgs::Twist cmd_vel_right;
  cmd_vel_right.linear.y = -0.3;
  cmd_publisher.publish(cmd_vel_right);
  sleep(3.0);


  geometry_msgs::Twist cmd_vel_turn_right;
  cmd_vel_turn_right.angular.z = -1;
  cmd_publisher.publish(cmd_vel_turn_right);
  sleep(3.0);

  geometry_msgs::Twist cmd_vel_turn_left;
  cmd_vel_turn_left.angular.z = 1;
  cmd_publisher.publish(cmd_vel_turn_left);
  sleep(3.0);

  return 0;
}
