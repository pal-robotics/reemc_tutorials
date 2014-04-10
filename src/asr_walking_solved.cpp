#include "ros/ros.h"
#include <pal_interaction_msgs/recognizerService.h>
#include <pal_interaction_msgs/asrresult.h>
#include <pal_interaction_msgs/SoundAction.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/bind.hpp>

#define NODE_NAME "pal_asr_walking"
#define ASR_TOPIC "/usersaid"
#define ASR_SERVICE "/asrservice"
#define ACTION_NAME "sound"


void say(const std::string &text, const std::string &langauge)
{
  /* create a sound action client */
  /* create a goal with the text to be said in the specified language */
  /* Send the goal */

  actionlib::SimpleActionClient<pal_interaction_msgs::SoundAction> ac(ACTION_NAME, true);

  ac.waitForServer(ros::Duration(10,0));

  if (!ac.isServerConnected())
  {
    ROS_ERROR("Unable to connect to server");
    exit(1);
  }

  pal_interaction_msgs::SoundGoal goal;
  goal.text = text;
  goal.wait_before_speaking = ros::Duration(0,0);
  ac.sendGoal(goal);
}


void asrResultsCallback(pal_interaction_msgs::asrresultConstPtr result)
{
  ROS_INFO_STREAM("You said: " << result->text);

  if (result->text == "step forward")
  {
    ROS_INFO("step forward");
    say("I stept forward", "en_GB");
  }
  else if (result->text == "step backward")
  {
    ROS_INFO("step backward");
    say("I stept backward", "en_GB");
  }
  else if (result->text == "turn right")
  {
    ROS_INFO("turn right");
    say("I turned right", "en_GB");
  }
  else if (result->text == "turn left")
  {
    ROS_INFO("turn left");
    say("I turned left", "en_GB");
  }
  else if (result->text == "move left")
  {
    ROS_INFO("move left");
    say("I moved left", "en_GB");
  }
  else if (result->text == "move right")
  {
    ROS_INFO("move right");
    say("I moved right", "en_GB");
  }

  /* Say some instructions for accepting a new command */
  say("Tell me what you want me to do next", "en_GB");

}



int main(int argc, char **argv)
{

  ros::init(argc, argv, NODE_NAME);
  std::string language = "en_US";
  std::string grammar = "walking";

  /* Create a ros node */
  ros::NodeHandle nh(NODE_NAME);

  /* create a service client and store it in client */
  ros::ServiceClient client = nh.serviceClient<pal_interaction_msgs::recognizerService>(ASR_SERVICE);

  /* create a subscribed and store it in subscriber */
  ros::Subscriber subscriber = nh.subscribe<pal_interaction_msgs::asrresult>(ASR_TOPIC, 0,&asrResultsCallback);

  /* Say some instructions to the user */
  say("You can command me with voice commands", "en_GB");

  /* construct a service message to send to the asrservice
   * in order to activate the ASR with walking grammar in the English language */
  pal_interaction_msgs::recognizerService srv;
  srv.request.asrupdate.language = language;
  srv.request.asrupdate.enable_grammar = grammar;
  srv.request.asrupdate.active = true;

  /* call the service */
  if (client.call(srv))
  {
    ROS_INFO("ASR enabled");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to call service " << ASR_SERVICE);
    exit(1);
  }

  ros::spin();

  return 0;
}
