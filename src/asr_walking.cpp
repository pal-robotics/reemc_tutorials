#include "ros/ros.h"
#include <pal_interaction_msgs/recognizerService.h>
#include <pal_interaction_msgs/asrresult.h>
#include <text_to_speech/SoundAction.h>
#include <actionlib/client/simple_action_client.h>
#include <controller_manager_msgs/SwitchController.h>
#include <geometry_msgs/Twist.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#define NODE_NAME "pal_asr_walking"
#define ASR_TOPIC "/usersaid"
#define ASR_SERVICE "/asrservice"
#define ACTION_NAME "sound"
#define STEP_TIME 2
#define WALK_CMD_VEL_TOPIC "/walking_controller/cmd_vel"
#define FWD_VEL 0.5
#define SIDE_VEL 0.3
#define TURN_VEL 1


void say(const std::string &text, const std::string &langauge)
{
  /* create a sound action client */
  /* create a goal with the text to be said in the specified language */
  /* Send the goal */

  actionlib::SimpleActionClient<text_to_speech::SoundAction> ac(ACTION_NAME, true);

  ac.waitForServer(ros::Duration(10,0));

  if (!ac.isServerConnected())
  {
    ROS_ERROR("Unable to connect to server");
    exit(1);
  }

  text_to_speech::SoundGoal goal;
  goal.text = text;
  goal.wait_before_speaking = ros::Duration(0,0);
  ac.sendGoal(goal);
  ac.waitForResult(ros::Duration(4,0));
}


void asrResultsCallback(pal_interaction_msgs::asrresultConstPtr result,  geometry_msgs::TwistPtr cmd_vel)
{
  ROS_INFO_STREAM("You said: " << result->text);


  if (result->text == "step forward")
  {
    cmd_vel->linear.x = FWD_VEL;
    sleep(STEP_TIME);
    say("I stept forward", "en_GB");
  }
  else if (result->text == "step backward")
  {
    cmd_vel->linear.x = -FWD_VEL;
    sleep(STEP_TIME);
    say("I stept backward", "en_GB");
  }
  else if (result->text == "move left")
  {
    cmd_vel->linear.y = SIDE_VEL;
    sleep(STEP_TIME);
    say("I moved left", "en_GB");
  }
  else if (result->text == "move right")
  {
    cmd_vel->linear.y = -SIDE_VEL;
    sleep(STEP_TIME);
    say("I moved right", "en_GB");
  }
  else if (result->text == "turn left")
  {
    cmd_vel->angular.z = TURN_VEL;
    sleep(STEP_TIME);
    say("I turned left", "en_GB");
  }
  else if (result->text == "turn right")
  {
    cmd_vel->angular.z = -TURN_VEL;
    sleep(STEP_TIME);
    say("I turned right", "en_GB");
  }

  cmd_vel->linear.x = 0;
  cmd_vel->linear.y = 0;
  cmd_vel->angular.z = 0;

  /* Say some instructions for accepting a new command */
  say("Tell me what you want me to do next", "en_GB");

}


void myRosSpin(geometry_msgs::TwistPtr cmd_vel)
//void myRosSpin()
{
  ros::NodeHandle nh(NODE_NAME);

  /* Create the walking publisher */
  ros::Publisher cmd_publisher = nh.advertise<geometry_msgs::Twist>(WALK_CMD_VEL_TOPIC,1 );

  while(ros::ok())
  {
    ros::spinOnce();
    sleep(0.5);
    cmd_publisher.publish(cmd_vel);
  }
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, NODE_NAME);
  std::string language = "en_US";
  std::string grammar = "walking";

  geometry_msgs::TwistPtr cmd_vel (new geometry_msgs::Twist());
  cmd_vel->linear.x = 0;
  cmd_vel->linear.y = 0;
  cmd_vel->angular.z = 0;

  /* Create a ros node */
  ros::NodeHandle nh(NODE_NAME);
  ROS_INFO_STREAM("Creating node " << NODE_NAME);

  /* create a service client and store it in client */
  ros::ServiceClient client = nh.serviceClient<pal_interaction_msgs::recognizerService>(ASR_SERVICE);



  /* create a subscribed and store it in subscriber */
  ros::Subscriber subscriber = nh.subscribe<pal_interaction_msgs::asrresult>(ASR_TOPIC, 0, boost::bind(asrResultsCallback,_1, cmd_vel));


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

  sleep(5);

  /* Say some instructions to the user */
  say("Now, you can command me with voice commands. You can say \"step forward\", \"step backward\", \"move right\", \"move left\", \"turn right\" or \"turn left\".", "en_GB");

  boost::thread p(boost::bind(myRosSpin, cmd_vel));

  while(ros::ok())
  {
    ros::spinOnce();
    sleep(0.5);
  }


  return 0;
}
