#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <humanoid_nav_msgs/ExecFootstepsAction.h>
#include <controller_manager_msgs/SwitchController.h>

const std::string WALK_STEPS_ACTION_NAME = "/walking_controller/footsteps_execution";
const double HIP_SPACING = (0.145 / 2.0);
const int nsteps = 5;
const double step_length = 0.15;
const double step_time = 1.0;

typedef actionlib::SimpleActionClient<humanoid_nav_msgs::ExecFootstepsAction> WalkingClient;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "walking_client_example");
  ros::NodeHandle n;


  WalkingClient action_client(WALK_STEPS_ACTION_NAME, true);
  if(!action_client.waitForServer(ros::Duration(5.0) ) )
  {
    ROS_ERROR_STREAM("Walking action server " << WALK_STEPS_ACTION_NAME << " not available. Check if walking controller has been loaded and started.");
    return 1;
  }

  humanoid_nav_msgs::ExecFootstepsGoal goal;

  // Create a list of steps
  humanoid_nav_msgs::StepTarget foot;
  for(unsigned int i=0; i <= nsteps; ++i){

    foot.leg = foot.leg == humanoid_nav_msgs::StepTarget::right?
          humanoid_nav_msgs::StepTarget::left : humanoid_nav_msgs::StepTarget::right;

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

  action_client.sendGoal(goal);
  action_client.waitForResult(ros::Duration(20.0));
  if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("The footstep list has been excecuted succesfully");
  }

  return 0;

}

