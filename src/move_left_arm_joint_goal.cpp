/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
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

// NOTE: The contents of this file are an adaptation of a similar tutorial on the moveit wiki:
// http://moveit.ros.org/wiki/MoveGroup_Interface

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_left_arm_joint_goal_test", ros::init_options::AnonymousName);
  // start a ROS spinning thread
  ros::AsyncSpinner spinner(1);
  spinner.start();
  // connect to a running instance of the move_group node
  move_group_interface::MoveGroup group("left_arm");
  ROS_INFO("Connected to server.");
  // set joint goal
  std::map< std::string, double > goal;
  goal["arm_left_1_joint"] = 1.7;
  goal["arm_left_2_joint"] = -0.1;
  goal["arm_left_3_joint"] = -1.7;
  goal["arm_left_4_joint"] =  1.5708;
  goal["arm_left_5_joint"] =  0.0;
  goal["arm_left_6_joint"] =  0.0;
  goal["arm_left_7_joint"] =  0.0;

  group.setJointValueTarget(goal);
  // set parameters for planning and execution
  group.setPlanningTime(5.0);
  group.setGoalJointTolerance(0.05);
  group.setGoalPositionTolerance(0.0);
  //group.setPlannerId("ompl_interface_ros/OMPLPlanner");
  //goal.motion_plan_request.num_planning_attempts = 1; // can we specify something equivalent to this?

  // plan the motion and then move the group to the sampled target
  bool success = group.move();

  if(success)
    ROS_INFO("Action succeeded.");
  else
    ROS_INFO("Action failed.");
  group.stop();
  ros::shutdown();
  return EXIT_SUCCESS;
}
