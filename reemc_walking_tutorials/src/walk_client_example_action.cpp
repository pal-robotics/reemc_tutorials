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
 *  Created on: 1/10/2014
 *      Author: Luca Marchionni
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <humanoid_nav_msgs/ExecFootstepsAction.h>

const std::string WALK_STEPS_ACTION_NAME = "/walking_controller/footsteps_execution";
const double HIP_SPACING = (0.145 / 2.0);
const unsigned int nsteps = 5;
const double step_length = 0.15;
const double step_time = 1.0;

typedef actionlib::SimpleActionClient<humanoid_nav_msgs::ExecFootstepsAction> WalkingClient;


void doneCb( const actionlib::SimpleClientGoalState& state, const humanoid_nav_msgs::ExecFootstepsResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

void activeCb()
{
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb( const humanoid_nav_msgs::ExecFootstepsFeedbackConstPtr& feedback)
{
  ROS_INFO_STREAM("Got Feedback : steps " << feedback->executed_footsteps.size() << " executed");
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "walking_client_example");
  ros::NodeHandle n;

  WalkingClient action_client(WALK_STEPS_ACTION_NAME, true);
  if(!action_client.waitForServer(ros::Duration(10.0) ) )
  {
    ROS_ERROR_STREAM("Walking action server " << WALK_STEPS_ACTION_NAME << " not available. Check if walking controller has been loaded and started.");
    return 1;
  }

  humanoid_nav_msgs::ExecFootstepsGoal goal;

  // Create a list of steps
  humanoid_nav_msgs::StepTarget foot;
  for(unsigned int i=0; i <= nsteps; ++i){

    if(foot.leg == humanoid_nav_msgs::StepTarget::right){
      foot.leg  = humanoid_nav_msgs::StepTarget::left;
    }
    else{
      foot.leg  = humanoid_nav_msgs::StepTarget::right;
     }

    if(i < nsteps )
    {
      foot.pose.x = step_length;
      foot.pose.y = -HIP_SPACING*(2.0 - 4*foot.leg);
      foot.pose.theta = 0;
    }
    else
    {
      // last step with zero lenght
      foot.pose.x = 0;
      foot.pose.y = -HIP_SPACING*(2.0 - 4*foot.leg);
      foot.pose.theta = 0;
    }

    std::cerr<<"foot pose "<<foot.pose.x<<" "<<foot.pose.y<<std::endl;
    goal.footsteps.push_back(foot);
  }

  goal.feedback_frequency = 1.0;

  action_client.sendGoal(goal, doneCb, activeCb, feedbackCb);

  action_client.waitForResult(ros::Duration(20.0));
  if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("The footstep list has been excecuted succesfully");
  }

  return 0;

}

