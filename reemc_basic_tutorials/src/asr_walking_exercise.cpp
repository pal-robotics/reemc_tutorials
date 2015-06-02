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
  /* FILL */
  /* create a sound action client */
  /* create a goal with the text to be said in the specified language */
  /* Send the goal */

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

  /* FILL */
  /* Say some instructions for accepting a new command */

}



int main(int argc, char **argv)
{

  ros::init(argc, argv, NODE_NAME);
  std::string language = "en_US";
  std::string grammar = "walking";

  /* FILL */
  /* Create a ros node */

  /* create a service client to manage the speech recognizer */

  /* create a subscribed that used asrResultsCallback as callback */

  /* Say some instructions to the user */


  /* construct a service message to send to the asrservice
   * in order to activate the ASR with walking grammar in the English language */

  /* call the service */


  ros::spin();

  return 0;
}
