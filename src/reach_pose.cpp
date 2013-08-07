/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2012, PAL Robotics, S.L.
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

/** \author Adolfo Rodriguez Tsouroukdissian. */
// NOTE: The contents of this file are based on the JointTrajectoryAction tutorial available here:
// http://www.ros.org/wiki/pr2_controllers/Tutorials/Moving%20the%20arm%20using%20the%20Joint%20Trajectory%20Action

#include <cassert>
#include <iostream>
#include <sstream>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>

/// \brief Move a joint group to a given pose.
///
/// The pose will be reached within a specified duration with zero velocity.
class MoveJointGroup
{
public:
  MoveJointGroup(const std::string& controller_name);
  bool sendGoal(const std::vector<double>& pose, const ros::Duration& duration);
  const std::vector<std::string>& getJointNames() const {return joint_names_;}
  actionlib::SimpleClientGoalState getState() {return client_.getState();}

private:
  typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ActionClient;
  typedef control_msgs::FollowJointTrajectoryGoal ActionGoal;

  std::string controller_name_;          ///< Controller name.
  std::vector<std::string> joint_names_; ///< Names of controller joints.
  ActionClient client_;                  ///< Action client used to trigger motions.

  bool configure();
};

MoveJointGroup::MoveJointGroup(const std::string& controller_name)
  : controller_name_(controller_name),
    client_(controller_name_ + "/follow_joint_trajectory")
{
  if (!configure())
  {
    throw;
  }
}

bool MoveJointGroup::configure()
{
  // Wait for the action client to be connected to the server
  if (!client_.waitForServer(ros::Duration(5.0))) // NOTE: Magic value
  {
    ROS_ERROR_STREAM("Timed-out waiting for the \'" << controller_name_ << "\' FollowJointTrajectory action server.");
    return false;
  }

  // Get list of joints used by the controller
  joint_names_.clear();
  using namespace XmlRpc;
  XmlRpcValue joint_names;
  ros::NodeHandle nh(controller_name_);
  if (!nh.getParam("joints", joint_names))
  {
    ROS_ERROR("No joints given. (namespace: %s)", nh.getNamespace().c_str());
    return false;
  }
  if (joint_names.getType() != XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Malformed joint specification.  (namespace: %s)", nh.getNamespace().c_str());
    return false;
  }
  for (int i = 0; i < joint_names.size(); ++i)
  {
    XmlRpcValue &name_value = joint_names[i];
    if (name_value.getType() != XmlRpcValue::TypeString)
    {
      ROS_ERROR("Array of joint names should contain all strings.  (namespace: %s)",
                nh.getNamespace().c_str());
      return false;
    }
    joint_names_.push_back(static_cast<std::string>(name_value));
  }
  return true;
}

bool MoveJointGroup::sendGoal(const std::vector<double>& pose, const ros::Duration& duration)
{
  // Goal pose for right_arm_torso group
  if (pose.size() != joint_names_.size())
  {
    ROS_ERROR_STREAM("Pose size mismatch. Expected: " << joint_names_.size() << ", got: " << pose.size() << ".");
    return false;
  }
  ActionGoal goal;
  goal.trajectory.joint_names = joint_names_;
  goal.trajectory.points.resize(1);
  goal.trajectory.points[0].positions = pose;                            // Reach these joint positions...
  goal.trajectory.points[0].velocities.resize(joint_names_.size(), 0.0); // ...with zero-velocity
  goal.trajectory.points[0].time_from_start = duration;                  // ...in this time

  client_.sendGoal(goal);
  return true;
}

/// \brief Move REEM's upper body to a given pose.
///
/// Poses are specified in the parameter server, and are identified by name.
class ReachPose
{
public:
  ReachPose(std::vector<std::string> controller_list, ros::NodeHandle& nh);

  /// \brief Send pose goal request and \e block until execution completes.
  /// \param pose Name of pose to execute.
  /// \param duration Motion duration.
  bool run(const std::string& pose, const ros::Duration& duration);

private:
  typedef boost::shared_ptr<MoveJointGroup> MoveJointGroupPtr;

  ros::NodeHandle nh_;
  std::vector<MoveJointGroupPtr> move_joint_groups_;
};

ReachPose::ReachPose(std::vector<std::string> controller_list, ros::NodeHandle& nh)
  : nh_(nh)
{
  if (controller_list.empty())
  {
    ROS_ERROR("Cannot initialize ReachPose instance, empty controller list provided.");
    throw;
  }
  BOOST_FOREACH(const std::string& controller_name, controller_list)
  {
    move_joint_groups_.push_back(MoveJointGroupPtr(new MoveJointGroup(controller_name)));
  }
}

bool ReachPose::run(const std::string& pose, const ros::Duration& duration)
{
  using namespace sensor_msgs;
  using namespace ros::topic;
  using namespace XmlRpc;
  typedef std::vector<std::string> JointNames;
  typedef std::vector<double> JointPositions;

  // Get current joint state
  JointStateConstPtr joint_state = waitForMessage<JointState>("/joint_states", ros::Duration(5.0)); // NOTE: Magic value
  if (!joint_state)
  {
    ROS_ERROR_STREAM("Timed-out waiting for the current joint state.");
    return false;
  }
  const JointNames&     joint_names     = joint_state->name;
  const JointPositions& joint_positions = joint_state->position;
  assert(joint_names.size() == joint_positions.size());

  std::vector<JointPositions> joint_group_pose; // Will contain desired pose split into joint groups

  // Verify that the pose specification exists
  if (!nh_.hasParam(pose))
  {
    ROS_ERROR_STREAM("Pose \'" << pose << "\' does not exist.");
    return false;
  }

  // Seed target pose with current joint state
  BOOST_FOREACH(MoveJointGroupPtr move_joint_group, move_joint_groups_)
  {
    JointNames     group_joint_names = move_joint_group->getJointNames();
    JointPositions group_joint_positions(group_joint_names.size());

    for (std::size_t i = 0; i < group_joint_names.size(); ++i)
    {
      // Fetch joint position value from pose specification
      if (!nh_.getParam(pose + "/" + group_joint_names[i], group_joint_positions[i]))
      {
        //...and if unspecified, use current joint position
        bool found = false;
        for (std::size_t j = 0; j < joint_names.size(); ++j)
        {
          if (group_joint_names[i] == joint_names[j])
          {
            group_joint_positions[i] = joint_positions[j];
            found = true;
            break;
          }
        }
        if (!found)
        {
          ROS_ERROR_STREAM("Could not get current position of joint \'" << group_joint_names[i] << "\'.");
          return false;
        }
      }
    }

    // Set pose goal for current joint group
    joint_group_pose.push_back(group_joint_positions);
  }

  // Send pose commands
  for (std::size_t i = 0; i < move_joint_groups_.size(); ++i)
  {
    move_joint_groups_[i]->sendGoal(joint_group_pose[i], duration);
  }

  // Wait until pose is reached
  while(ros::ok())
  {
    bool done = true;
    BOOST_FOREACH(MoveJointGroupPtr move_joint_group, move_joint_groups_)
    {
      done &= move_joint_group->getState().isDone();
    }
    if (done)
    {
      break;
    }
    else
    {
      ROS_INFO("Waiting for motion to complete...");
      ros::Duration(2.0).sleep();
    }
  }
  ROS_INFO_STREAM("Finished with state " << move_joint_groups_[0]->getState().toString());

  return true;
}

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "reach_pose");

  // Precondition: Pose name and (optionally) duration are provided as arguments
  std::string motion_name;
  const double default_duration = 5.0;
  double motion_duration = default_duration;
  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " pose_name [duration]\n"
        << "  where\n"
        << "  - \'pose_name\' is the name of a pose currently registered in the parameter server.\n"
        << "  - \'duration\' is the motion duration (in s). If unspecified, " << default_duration << "s will be used.";
  if (2 != argc && 3 != argc)
  {
    std::cout << usage.str() << std::endl;
    return EXIT_FAILURE;
  }

  if (3 == argc)
  {
    try
    {
      motion_duration = boost::lexical_cast<double>(argv[2]);
    }
    catch(boost::bad_lexical_cast &)
    {
      std::cout << usage.str() << std::endl;
      return EXIT_FAILURE;
    }
  }
  motion_name = argv[1];

  // Node handle scoped to where the poses are specified
  ros::NodeHandle nh_poses("~/poses");

  // Precondition: Valid clock
  if (!ros::Time::waitForValid(ros::WallDuration(5.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  // Controller names
  std::vector<std::string> controller_list;

  using namespace XmlRpc;
  XmlRpcValue controller_names;
  ros::NodeHandle nh_controllers("~");
  if (!nh_controllers.getParam("controllers", controller_names))
  {
    ROS_ERROR("No controllers given. (namespace: %s)", nh_controllers.getNamespace().c_str());
    return false;
  }
  if (controller_names.getType() != XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Malformed controller specification.  (namespace: %s)", nh_controllers.getNamespace().c_str());
    return false;
  }
  for (int i = 0; i < controller_names.size(); ++i)
  {
    XmlRpcValue &name_value = controller_names[i];
    if (name_value.getType() != XmlRpcValue::TypeString)
    {
      ROS_ERROR("Array of controller names should contain all strings.  (namespace: %s)",
                nh_controllers.getNamespace().c_str());
      return false;
    }
    controller_list.push_back(static_cast<std::string>(name_value));
  }

  // Send motion execution request
  ReachPose reach_pose(controller_list, nh_poses);

  return reach_pose.run(motion_name, ros::Duration(motion_duration)) ? EXIT_SUCCESS : EXIT_FAILURE;
}