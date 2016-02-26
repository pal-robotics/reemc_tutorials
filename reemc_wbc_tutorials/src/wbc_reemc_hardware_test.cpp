/*
* Software License Agreement (Modified BSD License)
*
* Copyright (c) 2014, 2015 PAL Robotics, S.L.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of PAL Robotics, S.L. nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

/*
* Author: Hilario Tomé
*/

#include <Eigen/Dense>
#include <ros/ros.h>
#include <rbdl/rbdl_utils.h>
#include <Eigen/Dense>
#include <pal_robot_tools/reference/reference_abstract.h>
#include <pal_robot_tools/walking_visualization_tools.h>

/**
 * The following example publishes a list of goals to the left hand, right hand and head gaze
 * topic corresponding to the respective tasks. Its used to try in simulation and the real
 * robot the whole body controller
 */

ros::Publisher head_goal_pub;
ros::Publisher left_arm_pub;
ros::Publisher right_arm_pub;

ros::Publisher marker_pub;

using namespace Eigen;

void publishGoal(eVector3 head, eVector3 left_arm, eVector3 right_arm){

  geometry_msgs::PoseStamped head_target;
  head_target.header.frame_id = "/world";
  head_target.header.stamp = ros::Time::now();
  head_target.pose.position.x = head(0);
  head_target.pose.position.y = head(1);
  head_target.pose.position.z = head(2);

  geometry_msgs::PoseStamped left_arm_target;
  left_arm_target.header.frame_id = "/world";
  left_arm_target.header.stamp = ros::Time::now();
  left_arm_target.pose.position.x = left_arm(0);
  left_arm_target.pose.position.y = left_arm(1);
  left_arm_target.pose.position.z = left_arm(2);

  geometry_msgs::PoseStamped right_arm_target;
  right_arm_target.header.frame_id = "/world";
  right_arm_target.header.stamp = ros::Time::now();
  right_arm_target.pose.position.x = right_arm(0);
  right_arm_target.pose.position.y = right_arm(1);
  right_arm_target.pose.position.z = right_arm(2);

  head_goal_pub.publish(head_target);
  left_arm_pub.publish(left_arm_target);
  right_arm_pub.publish(right_arm_target);

  visualization_msgs::MarkerArray marray;
  marray.markers.reserve(3);
  unsigned int index = 0;
  Eigen::Vector3d red(1, 0, 0);

  publish_sphere(head, red, 0.05, "world", "head_target", ros::Time::now(), marray, index);
  publish_sphere(left_arm, red, 0.05, "world", "left_arm_target", ros::Time::now(), marray, index);
  publish_sphere(right_arm, red, 0.05, "world", "right_arm_target", ros::Time::now(), marray, index);

  marker_pub.publish(marray);
  marray.markers.clear();

}

int main(int argc, char** argv){

  ros::init(argc, argv, "wbc_reemc_hardware_test");
  ros::NodeHandle nh("~");

  double wait_time = 2.0;

  head_goal_pub =  nh.advertise<geometry_msgs::PoseStamped>("/whole_body_kinematic_controler/gaze_objective_stereo_optical_frame_goal", 10);
  right_arm_pub = nh.advertise<geometry_msgs::PoseStamped>("/whole_body_kinematic_controler/arm_right_7_link_goal", 10);
  left_arm_pub = nh.advertise<geometry_msgs::PoseStamped>("/whole_body_kinematic_controler/arm_left_7_link_goal", 10);

  marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/wbc_test_hardware_markers", 10);

  ros::Duration(wait_time).sleep();


  // Hand moving in the front
  ROS_INFO_STREAM("Goal: 1");
  publishGoal(eVector3(0.6, 0.0, 0.6), eVector3(0.4, 0.2, 0.0), eVector3(0.4, -0.2, 0.0));
  ros::Duration(wait_time).sleep();

  ROS_INFO_STREAM("Goal: 2");
  publishGoal(eVector3(0.6, 0.3, 0.6), eVector3(0.6, 0.2, 0.0), eVector3(0.6, -0.2, 0.0));
  ros::Duration(wait_time).sleep();

  ROS_INFO_STREAM("Goal: 3");
  publishGoal(eVector3(0.6, -0.3, 0.6), eVector3(0.3, 0.2, -0.2), eVector3(0.3, -0.2, 0.2));
  ros::Duration(wait_time).sleep();

  ROS_INFO_STREAM("Goal: 4");
  publishGoal(eVector3(0.6, 0.0, 1.0), eVector3(0.45, 0.2, 0.0), eVector3(0.45, -0.2, 0.0));
  ros::Duration(wait_time).sleep();

  ROS_INFO_STREAM("Goal: 5");
  publishGoal(eVector3(1.0, 0.0, 0.2), eVector3(0.6, 0.2, 0.3), eVector3(0.6, -0.2, -0.3));
  ros::Duration(wait_time).sleep();

  ROS_INFO_STREAM("Goal: 6");
  publishGoal(eVector3(0.6, 0.0, 0.6), eVector3(0.3, 0.2, 0.0), eVector3(0.3, -0.2, 0.0));
  ros::Duration(wait_time).sleep();

  ROS_INFO_STREAM("Goal: 7");
  publishGoal(eVector3(0.5, -0.5, 0.6), eVector3(0.4, 0.1, 0.0), eVector3(-0.4, -0.2, 0.0));
  ros::Duration(wait_time).sleep();
  ros::Duration(wait_time).sleep();

  ROS_INFO_STREAM("Goal: 8");
  publishGoal(eVector3(0.5, 0.0, 0.6), eVector3(0.0, 0.3, -0.2), eVector3(0.0, -0.3, -0.2));
  ros::Duration(wait_time).sleep();

  ROS_INFO_STREAM("Goal: 9");
  publishGoal(eVector3(0.5, 0.5, 0.6), eVector3(-0.4, 0.1, -0.2), eVector3(0.4, -0.1, -0.2));
  ros::Duration(wait_time).sleep();

  ROS_INFO_STREAM("Goal: 9");
  publishGoal(eVector3(0.5, 0.0, 0.6), eVector3(0.0, 0.3, -0.2), eVector3(0.0, -0.3, -0.2));
  ros::Duration(wait_time).sleep();

  ROS_INFO_STREAM("Goal: 10");
  publishGoal(eVector3(0.5, 0.0, 0.6), eVector3(0.3, 0.3, 0.2), eVector3(0.3, -0.3, 0.2));
  ros::Duration(wait_time).sleep();

  ROS_INFO_STREAM("Goal: 11");
  publishGoal(eVector3(0.5, -0.5, 0.8), eVector3(0.4, 0.2, 0.0), eVector3(-0.4, -0.2, 0.0));
  ros::Duration(wait_time).sleep();
  ros::Duration(wait_time).sleep();

  ROS_INFO_STREAM("Goal: 12");
  publishGoal(eVector3(0.5, 0.0, 0.6), eVector3(0.0, 0.3, 0.0), eVector3(0.0, -0.3, 0.0));
  ros::Duration(wait_time).sleep();

  ROS_INFO_STREAM("Goal: 13");
  publishGoal(eVector3(0.5, 0.5, 0.8), eVector3(-0.4, 0.1, 0.2), eVector3(0.4, -0.1, 0.2));
  ros::Duration(wait_time).sleep();

  ROS_INFO_STREAM("Goal: 14");
  publishGoal(eVector3(0.5, 0.0, 0.6), eVector3(0.0, 0.3, -0.2), eVector3(0.0, -0.3, -0.2));
  ros::Duration(wait_time).sleep();

  ROS_INFO_STREAM("Goal: 15");
  publishGoal(eVector3(0.5, 0.0, 0.8), eVector3(0.3, 0.3, 0.2), eVector3(0.3, -0.3, 0.2));
  ros::Duration(wait_time).sleep();

  ros::Duration(5.0).sleep();


}

