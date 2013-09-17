#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <humanoid_nav_msgs/ExecFootstepsAction.h>
#include <controller_manager_msgs/SwitchController.h>

const std::string WALK_STEPS_ACTION_NAME = "/footsteps_execution";
const double HIP_SPACING = (0.145 / 2.0);
const int nsteps = 5;
const double step_length = 0.15;
const double step_time = 1.0;

typedef actionlib::SimpleActionClient<humanoid_nav_msgs::ExecFootstepsAction> WalkingClient;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "walking_client_example");
  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller", true);

  controller_manager_msgs::SwitchController srv_start;
  srv_start.request.start_controllers.resize(1, "walking_controller");
  srv_start.request.stop_controllers.clear();
  srv_start.request.strictness = controller_manager_msgs::SwitchControllerRequest::STRICT;
  if (client.call(srv_start))
  {
    ROS_INFO("Succesfully started walking controller");
  }
  else
  {
    ROS_ERROR("Failed to start walking controller");
    return 1;
  }


  WalkingClient action_client(WALK_STEPS_ACTION_NAME, true);
  action_client.waitForServer();
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

  controller_manager_msgs::SwitchController srv_stop;
  srv_stop.request.stop_controllers.resize(1, "walking_controller");
  srv_stop.request.start_controllers.clear();
  srv_stop.request.strictness = controller_manager_msgs::SwitchControllerRequest::STRICT;
  if (client.call(srv_stop))
  {
    ROS_INFO("Succesfully stopped walking controller");
  }
  else
  {
    ROS_ERROR("Failed to stop walking controller");
    return 1;
  }


  return 0;

}

