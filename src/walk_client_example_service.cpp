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


  ros::Duration(5.0).sleep();

  ros::ServiceClient walking_client = n.serviceClient<walking::WalkSteps>(WALK_STEPS_SERVICE);
  if(! walking_client.waitForExistence(ros::Duration(5.0)) )
  {
    ROS_ERROR_STREAM("Walking service " << WALK_STEPS_SERVICE << " not available. Check if walking controller has been loaded and started.");
    return 1;
  }

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

  return 0;
}

