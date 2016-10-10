#include <iostream>
#include <eigen_checks/gtest.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <walking_msgs/WalkSteps.h>
#include <gazebo_msgs/LinkStates.h>
#include <actionlib/client/simple_action_client.h>
#include <humanoid_nav_msgs/ExecFootstepsAction.h>
#include <geometry_msgs/Twist.h>
#include <pal_robot_tools/math_utils.h>
#include <reemc_walking_tutorials/LinkStatesListener.h>

typedef actionlib::SimpleActionClient<humanoid_nav_msgs::ExecFootstepsAction> WalkingAction;

class Fixture: public ::testing::Test
{
protected:

  virtual void SetUp() // Initialization
  {
    links_listening_.push_back("reemc_full_ft_hey5::leg_left_6_link");
    links_listening_.push_back("reemc_full_ft_hey5::leg_right_6_link");
    link_listener_ptr_.reset(new LinkStatesListener(nh_, links_listening_));
    link_listener_ptr_->wait_until_Listener_ready();
  }

  virtual void TearDown() // Destruction
  {
    link_listener_ptr_.reset();
  }

  std::vector<std::string> links_listening_;
  boost::shared_ptr<LinkStatesListener> link_listener_ptr_;
  ros::NodeHandle nh_;
};

// Test that makes 5 steps using a service
TEST_F(Fixture, Move_forward_five_steps_service)
{
  ros::ServiceClient step_service = nh_.serviceClient<walking_msgs::WalkSteps>("/walking_controller/walk_steps");

  // Create the msg
  walking_msgs::WalkSteps msg;
  msg.request.nsteps = 5;
  msg.request.step_length = 0.25;
  msg.request.step_time = 1.0;

  // Store the expected pose (step_length * n_steps)
  Eigen::Vector3d expected_pose_1 =
      link_listener_ptr_->getMapPosition(links_listening_[0]);
  Eigen::Vector3d expected_pose_2 =
      link_listener_ptr_->getMapPosition(links_listening_[1]);

  expected_pose_1[0] += msg.request.nsteps*msg.request.step_length;
  expected_pose_2[0] += msg.request.nsteps*msg.request.step_length;

  if(! step_service.waitForExistence(ros::Duration(10.0)))
  {
    FAIL() << "Could not create the service";
  }

  step_service.call(msg);

  ros::Time start_time = ros::Time::now();

  // Wait until the service has finished (non blocking service) ATT! 4 is random choosed
  while(ros::Time::now()-start_time <
        ros::Duration(msg.request.step_time*msg.request.nsteps*4))
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  // Compare the final pose with the expected one
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(expected_pose_1,
                                link_listener_ptr_->getMapPosition(links_listening_[0]),
                                2.e-1));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(expected_pose_2,
                                link_listener_ptr_->getMapPosition(links_listening_[1]),
                                2.e-1));
}

//Test that makes 5 steps using an action
TEST_F(Fixture, Move_forward_five_steps_action)
{
  // Default paramaters
  size_t nsteps = 5;
  const double step_length = 0.25;
  const double HIP_SPACING = (0.145 / 2.0);

  // Store the expected pose (step_length * nsteps)
  Eigen::Vector3d expected_pose_1 =
      link_listener_ptr_->getMapPosition(links_listening_[0]);
  Eigen::Vector3d expected_pose_2 =
      link_listener_ptr_->getMapPosition(links_listening_[1]);

  expected_pose_1[0] += nsteps*step_length;
  expected_pose_2[0] += nsteps*step_length;

  WalkingAction action("/walking_controller/footsteps_execution",true);

  if(!action.waitForServer(ros::Duration(10.0)))
  {
   FAIL() << "Could not create action";
  }

  // Create the StepList message
  humanoid_nav_msgs::ExecFootstepsGoal goal;

  humanoid_nav_msgs::StepTarget foot;
  for(unsigned int i=0; i <= nsteps; ++i)
  {
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
      foot.pose.x = 0;
      foot.pose.y = -HIP_SPACING*(2.0 - 4*foot.leg);
      foot.pose.theta = 0;
    }
    goal.footsteps.push_back(foot);
  }
  goal.feedback_frequency = 1.0;

  action.sendGoal(goal);

  // Wait until action has finished
  action.waitForResult(ros::Duration(30.0));

  ros::spinOnce();

  // If action succeed compare the final position with the expected one
  if (action.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(expected_pose_1,
                                  link_listener_ptr_->getMapPosition(links_listening_[0]),
                                  2.e-1));
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(expected_pose_2,
                                  link_listener_ptr_->getMapPosition(links_listening_[1]),
                                  2.e-1));
  }
  else
  {
    FAIL() << "Action not succeed";
  }
}

// Test that creates different kind of steps bu publishing velocities
TEST_F(Fixture, Move_forward_five_steps_topic)
{
  // Create the publisher
  ros::Publisher cmd_publisher = nh_.advertise<geometry_msgs::Twist>
                                 ("/walking_controller/cmd_vel", 1);

  // Stores the expected pose (as a goal)
  Eigen::Vector3d expected_pose_1 = link_listener_ptr_->getMapPosition(links_listening_[0]);
  Eigen::Vector3d expected_pose_2 = link_listener_ptr_->getMapPosition(links_listening_[1]);

  expected_pose_1[0] += 0.5;
  expected_pose_2[0] += 0.5;

  ros::Time start_time = ros::Time::now();
  bool dist_succeed_1 = false;
  bool dist_succeed_2 = false;
  bool dist_succeed_3 = false;

  // Move forward
  geometry_msgs::Twist cmd_vel_forward;
  cmd_vel_forward.linear.x = 1.0;

  while((ros::Time::now()-start_time) < ros::Duration(25.0) && !(dist_succeed_1))
  {
    // If reaches the goal before time... dist_succeed_1 = true
    if(expected_pose_1[0] < link_listener_ptr_->getMapPosition(links_listening_[0])[0]
       && expected_pose_2[0] < link_listener_ptr_->getMapPosition(links_listening_[1])[0])
    {
      dist_succeed_1 = true;
    }
    cmd_publisher.publish(cmd_vel_forward);
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  start_time = ros::Time::now();

  // Move left
  geometry_msgs::Twist cmd_vel_left;
  cmd_vel_left.linear.y = 1.5;
  cmd_publisher.publish(cmd_vel_left);

  // Stores the lateral goal
  expected_pose_1[1] += 0.4;
  expected_pose_2[1] += 0.4;

  while((ros::Time::now()-start_time) < ros::Duration(20.0) && !(dist_succeed_2))
  {
    // If reaches the goal before time... dist_succeed_2 = true
    if(expected_pose_1[1] < link_listener_ptr_->getMapPosition(links_listening_[0])[1]
       && expected_pose_2[1] < link_listener_ptr_->getMapPosition(links_listening_[0])[1])
    {
      dist_succeed_2 = true;
    }
    cmd_publisher.publish(cmd_vel_left);
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  start_time = ros::Time::now();

  // Turn
  geometry_msgs::Twist cmd_vel_turn_left;
  cmd_vel_turn_left.angular.z = 1.5;
  cmd_publisher.publish(cmd_vel_turn_left);

  Eigen::Quaterniond init_orient_0 = link_listener_ptr_->getMapOrientation(links_listening_[0]);
  Eigen::Quaterniond init_orient_1 = link_listener_ptr_->getMapOrientation(links_listening_[1]);



  while((ros::Time::now()-start_time) < ros::Duration(30.0) && !(dist_succeed_3))
  {
    Eigen::Quaterniond curr_orient_0 = link_listener_ptr_->getMapOrientation(links_listening_[0]);
    Eigen::Quaterniond curr_orient_1 = link_listener_ptr_->getMapOrientation(links_listening_[1]); 
       
    double diff_angle_0 = acos((2*pow((curr_orient_0.dot(init_orient_0)),2))-1);
    double diff_angle_1 = acos((2*pow((curr_orient_1.dot(init_orient_1)),2))-1);

    if(diff_angle_0 > M_PI/2 && diff_angle_1 > M_PI/2)
    {
      dist_succeed_3 = true;
    }
    cmd_publisher.publish(cmd_vel_turn_left);
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  // Expect that all the dist_succeed should be true
  EXPECT_TRUE(dist_succeed_1);
  EXPECT_TRUE(dist_succeed_2);
  EXPECT_TRUE(dist_succeed_3);
}


int main(int argc, char** argv)
{
  ros::init (argc, argv, "five_steps_test");

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
