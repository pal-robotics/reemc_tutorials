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
#include <walking_msgs/WalkSteps.h>

const std::string WALK_STEPS_SERVICE = "/walking_controller/walk_steps";
const int nsteps = 5;
const double step_length = 0.15;
const double step_time = 1.0;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "walking_client_example");
  ros::NodeHandle n;


  ros::Duration(5.0).sleep();

  ros::ServiceClient walking_client = n.serviceClient<walking_msgs::WalkSteps>(WALK_STEPS_SERVICE);
  if(! walking_client.waitForExistence(ros::Duration(5.0)) )
  {
    ROS_ERROR_STREAM("Walking service " << WALK_STEPS_SERVICE << " not available. Check if walking controller has been loaded and started.");
    return 1;
  }

  walking_msgs::WalkSteps srv;
  srv.request.nsteps = nsteps;
  srv.request.step_length = step_length;
  srv.request.step_time = step_time;

  if (walking_client.call(srv))
  {
    ROS_INFO("Succesfully called service WalkSteps");
  }
  else
  {
    ROS_ERROR("Failed to call service WalkSteps");
    return 1;
  }

  ros::Duration(step_time*nsteps + 5.0).sleep();
  srv.request.step_length = -step_length;

  if (walking_client.call(srv))
  {
    ROS_INFO("Succesfully called service WalkSteps");
  }
  else
  {
    ROS_ERROR("Failed to call service WalkSteps");
    return 1;
  }
  ros::Duration(step_time*nsteps + 10.0).sleep();

  return 0;
}

