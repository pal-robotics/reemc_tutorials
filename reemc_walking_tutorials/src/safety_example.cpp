/*
 *  safety_example.cpp
 *  Copyright (c) 2015 PAL Robotics sl. All Rights Reserved
 *  Created on: 6/5/2015
 *      Author: luca
 */


#include <ros/ros.h>
#include <walking_controller/safety.hpp>
#include <sensor_msgs/JointState.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "safety_example");
  ros::NodeHandle nh;

  sensor_msgs::JointState lower_body_joint_states;
  /// Setup the joint states message for only the legs
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

  ros::Duration dT(0.01);
  pal::BipedSafety biped_safety(&nh, &lower_body_joint_states, dT);


  ros::Time time(0.0);
  bool is_safe = biped_safety.is_safe(lower_body_joint_states.position, time);

  if(is_safe)
    ROS_INFO_STREAM("First configuration is safe");
  else
    ROS_INFO_STREAM("First configuration is not safe");


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
  is_safe = biped_safety.is_safe(lower_body_joint_states.position, time);

  if(is_safe)
    ROS_INFO_STREAM("Second configuration is safe");
  else
    ROS_INFO_STREAM("Second configuration is not safe");

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
  is_safe = biped_safety.is_safe(lower_body_joint_states.position, time);

  if(is_safe)
    ROS_INFO_STREAM("Third configuration is safe");
  else
    ROS_INFO_STREAM("Third configuration is not safe");

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

  is_safe = biped_safety.velocity_limits_respected(lower_body_joint_states.position);
  if(is_safe)
    ROS_INFO_STREAM("Fourth configuration is safe");
  else
    ROS_INFO_STREAM("Fourth configuration is not safe");


  return 0;

}
