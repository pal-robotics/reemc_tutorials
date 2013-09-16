#include <ros/ros.h>
#include <controller_manager_msgs/SwitchController.h>
#include <geometry_msgs/Twist.h>

const std::string WALK_CMD_VEL_TOPIC = "/walking_controller/cmd_vel";

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

  ros::Publisher cmd_publisher = n.advertise<geometry_msgs::Twist>(WALK_CMD_VEL_TOPIC,1 );
  sleep(5);


  /// Sending a positive linear speed cmd for performing a forward step
  geometry_msgs::Twist cmd_vel_forward;
  cmd_vel_forward.linear.x = 0.5;
  cmd_publisher.publish(cmd_vel_forward);
  sleep(3.0);

  geometry_msgs::Twist cmd_vel_backward;
  cmd_vel_backward.linear.x = -0.5;
  cmd_publisher.publish(cmd_vel_backward);
  sleep(3.0);

  geometry_msgs::Twist cmd_vel_left;
  cmd_vel_left.linear.y = 0.3;
  cmd_publisher.publish(cmd_vel_left);
  sleep(3.0);

  geometry_msgs::Twist cmd_vel_right;
  cmd_vel_right.linear.y = -0.3;
  cmd_publisher.publish(cmd_vel_right);
  sleep(3.0);


  geometry_msgs::Twist cmd_vel_turn_right;
  cmd_vel_turn_right.angular.z = -1;
  cmd_publisher.publish(cmd_vel_turn_right);
  sleep(3.0);

  geometry_msgs::Twist cmd_vel_turn_left;
  cmd_vel_turn_left.angular.z = 1;
  cmd_publisher.publish(cmd_vel_turn_left);
  sleep(3.0);


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

