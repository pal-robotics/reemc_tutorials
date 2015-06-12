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

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <play_motion_msgs/PlayMotionAction.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_left_arm_without_planning", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  bool use_sim_time = false;
  nh.param<bool>("/use_sim_time", use_sim_time, false);
  if(!use_sim_time)
  {
    ROS_FATAL("/use_sim_time is not set or false and you are trying to run a dangerous node. Supposing you are executing this on the robot, exiting.");
    return EXIT_FAILURE;
  }
  if (!ros::Time::waitForValid(ros::WallDuration(5.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  actionlib::SimpleActionClient<play_motion_msgs::PlayMotionAction> pmClient("/play_motion", true);
  ROS_INFO("Connecting to server...");

  if(!pmClient.waitForServer(ros::Duration(10.0)))
  {
    ROS_ERROR_STREAM("Timed-out waiting for the play_motion server.");
    return EXIT_FAILURE;
  }
  ROS_INFO("Connected to server.");

  play_motion_msgs::PlayMotionGoal goal;
  goal.motion_name = "raise_left_hand";
  goal.skip_planning = true;

  if(nh.ok())
  {
    pmClient.sendGoal(goal);
    bool finished_within_time = pmClient.waitForResult(ros::Duration(15.0));
    if (!finished_within_time)
    {
      pmClient.cancelGoal();
      ROS_INFO("Timed out achieving joint-space goal.");
    }
    else
    {
      actionlib::SimpleClientGoalState state = pmClient.getState();
      bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      if(success)
        ROS_INFO("Action finished: %s",state.toString().c_str());
      else
        ROS_INFO("Action failed: %s",state.toString().c_str());
     }
  }

  ros::shutdown();
  return EXIT_SUCCESS;
}
