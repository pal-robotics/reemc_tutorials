#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <humanoid_nav_msgs/ExecFootstepsAction.h>
#include <sstream>

typedef actionlib::SimpleActionClient<humanoid_nav_msgs::ExecFootstepsAction> Client;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "walking_client_example");
  /*
  Client client("do_dishes", true); // true -> don't need ros::spin()
  client.waitForServer();
  chores::DoDishesGoal goal;
  // Fill in goal here
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The dishes are now clean");
  printf("Current State: %s\n", client.getState().toString().c_str());
  return 0;
*/
}

