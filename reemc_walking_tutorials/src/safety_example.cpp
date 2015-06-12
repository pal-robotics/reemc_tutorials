/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2015, PAL Robotics, S.L.
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
 *  safety_example.cpp
 *  Created on: 6/5/2015
 *      Author: Luca Marchionni
 */


#include <ros/ros.h>
#include <walking_controller/safety.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "safety_example");
  ros::NodeHandle nh;

  sensor_msgs::JointState lower_body_joint_states;

  /// Setup joint states message for leg joints
  lower_body_joint_states.name.resize(12);
  lower_body_joint_states.position.resize(12,0.0);
  lower_body_joint_states.velocity.resize(12, 0.0);
  lower_body_joint_states.effort.resize(12), 0.0;

  lower_body_joint_states.name[0] = "leg_left_1_joint";
  lower_body_joint_states.name[1] = "leg_left_2_joint";
  lower_body_joint_states.name[2] = "leg_left_3_joint";
  lower_body_joint_states.name[3] = "leg_left_4_joint";
  lower_body_joint_states.name[4] = "leg_left_5_joint";
  lower_body_joint_states.name[5] = "leg_left_6_joint";
  lower_body_joint_states.name[6] = "leg_right_1_joint";
  lower_body_joint_states.name[7] = "leg_right_2_joint";
  lower_body_joint_states.name[8] = "leg_right_3_joint";
  lower_body_joint_states.name[9] = "leg_right_4_joint";
  lower_body_joint_states.name[10] = "leg_right_5_joint";
  lower_body_joint_states.name[11] = "leg_right_6_joint";

  /// Control loop duration (10 ms) only used for velocity estimation
  ros::Duration dT(0.01);

  /// Creation of BipedSafety object
  pal::BipedSafety biped_safety(&nh, &lower_body_joint_states, dT);

  /// init time
  ros::Time time(0.0);

  /// Check for safety of the starting configuration (all joints at zero)
  bool is_safe = biped_safety.is_safe(lower_body_joint_states.position, time);

  if(is_safe)
    ROS_INFO_STREAM("First configuration is safe");
  else
    ROS_INFO_STREAM("First configuration is not safe");


  /// Setting a joints configuration that would cause a self collision
  lower_body_joint_states.position[0] = 0.0;
  lower_body_joint_states.position[1] = -0.25;
  lower_body_joint_states.position[2] = 0.0;
  lower_body_joint_states.position[3] = 0.0;
  lower_body_joint_states.position[4] = 0.0;
  lower_body_joint_states.position[5] = 0.0;
  lower_body_joint_states.position[6] = 0.0;
  lower_body_joint_states.position[7] = 0.25;
  lower_body_joint_states.position[8] = 0.0;
  lower_body_joint_states.position[9] = 0.0;
  lower_body_joint_states.position[10] = 0.0;
  lower_body_joint_states.position[11] = 0.0;

  time+=dT;

  /// Check for collision
  is_safe = biped_safety.is_safe(lower_body_joint_states.position, time);

  if(is_safe)
    ROS_INFO_STREAM("Second configuration is safe");
  else
    ROS_INFO_STREAM("Second configuration is not safe");

  /// Setting a joint position outside joint limit
  lower_body_joint_states.position[0] = 0.0;
  lower_body_joint_states.position[1] = -0.3;
  lower_body_joint_states.position[2] = 0.0;
  lower_body_joint_states.position[3] = 0.0;
  lower_body_joint_states.position[4] = 0.0;
  lower_body_joint_states.position[5] = 0.0;
  lower_body_joint_states.position[6] = 0.0;
  lower_body_joint_states.position[7] = 0.0;
  lower_body_joint_states.position[8] = 0.0;
  lower_body_joint_states.position[9] = 0.0;
  lower_body_joint_states.position[10] = 0.0;
  lower_body_joint_states.position[11] = 0.0;

  time+=dT;
  /// Check for safety (one joint will violate its position limit)
  is_safe = biped_safety.is_safe(lower_body_joint_states.position, time);

  if(is_safe)
    ROS_INFO_STREAM("Third configuration is safe");
  else
    ROS_INFO_STREAM("Third configuration is not safe");

  /// Configuration that would violate speed limit for 4th left leg joint
  lower_body_joint_states.position[0] = 0.0;
  lower_body_joint_states.position[1] = 0.0;
  lower_body_joint_states.position[2] = 0.0;
  lower_body_joint_states.position[3] = 1.2;
  lower_body_joint_states.position[4] = 0.0;
  lower_body_joint_states.position[5] = 0.0;
  lower_body_joint_states.position[6] = 0.0;
  lower_body_joint_states.position[7] = 0.0;
  lower_body_joint_states.position[8] = 0.0;
  lower_body_joint_states.position[9] = 0.0;
  lower_body_joint_states.position[10] = 0.0;
  lower_body_joint_states.position[11] = 0.0;

  /// Check for speed limit violations
  is_safe = biped_safety.velocity_limits_respected(lower_body_joint_states.position);
  if(is_safe)
    ROS_INFO_STREAM("Fourth configuration is safe");
  else
    ROS_INFO_STREAM("Fourth configuration is not safe");


  return 0;

}
