#include "ros/ros.h"
#include <pal_interaction_msgs/recognizerService.h>
#include <pal_interaction_msgs/asrresult.h>
#include <cstdlib>


void asrResultsCallback(const pal_interaction_msgs::asrresult result)
{
  ROS_INFO_STREAM("You said: " << result.text);
}


int main(int argc, char **argv)
{
  std::string serviceName = "asrservice";
  std::string topicName = "usersaid";
  std::string nodeName = "pal_asr_client";

  ros::init(argc, argv, nodeName);

  if (argc != 4)
  {
    ROS_INFO("usage: <language> <grammarname> <active>");
    return 1;
  }

  std::string language = argv[1];
  std::string grammarName = argv[2];
  std::string sactive = argv[3];
  bool active = (sactive == "true") ? true : false;

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<pal_interaction_msgs::recognizerService>(serviceName);
  ros::Subscriber subscriber = n.subscribe(topicName, 0, &asrResultsCallback);
  pal_interaction_msgs::recognizerService srv;

  srv.request.asrupdate.language = language;
  srv.request.asrupdate.enable_grammar = grammarName;
  srv.request.asrupdate.active = active;

  if (client.call(srv))
  {
    ROS_INFO("Called finished");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to call service " << serviceName);
    return 1;
  }

  ROS_INFO("Same some sentences included in the UAT grammar example.");
  ROS_INFO("To close the test type Ctrl+C");

  ros::spin();

  return 0;
}
