#include <ros/ros.h>
#include <controller_manager_msgs/SwitchController.h>
#include <walking/WalkSteps.h>

const std::string WALK_STEPS_SERVICE = "/walking_controller/walk_steps";
const int nsteps = 5;
const double step_length = 0.15;
const double step_time = 1.0;

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

  ros::Duration(5.0).sleep();

  ros::ServiceClient walking_client = n.serviceClient<walking::WalkSteps>(WALK_STEPS_SERVICE);
  walking::WalkSteps srv;
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

  ros::Duration(5.0).sleep();

  return 0;
}

