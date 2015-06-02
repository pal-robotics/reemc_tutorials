#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pal_interaction_msgs/SoundAction.h>
#include <cstdlib>

int main (int argc, char **argv)
{

  int time_to_wait = 0;
  if (argc < 2)
  {
    std::cout << "USE: " << argv[0] << " \"text to say\" [time to wait (ms)]" << std::endl;
    exit(1);
  }

  if (argc > 2)
  {
    time_to_wait = atoi(argv[2]);
  }

  std::string clientName = "pal_tts_client";
  std::string actionName = "sound";
  ros::init(argc, argv, clientName);

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<pal_interaction_msgs::SoundAction> ac(actionName, true);

  if (!ac.isServerConnected())
  {
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer(ros::Duration(10,0));
  }

  if (!ac.isServerConnected())
  {
    ROS_ERROR("Unable to connect to server");
    exit(1);
  }

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  pal_interaction_msgs::SoundGoal goal;
  goal.text = argv[1];
  goal.wait_before_speaking = ros::Duration(time_to_wait*1000,0);
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
