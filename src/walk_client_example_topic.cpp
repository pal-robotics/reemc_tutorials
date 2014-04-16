#include <ros/ros.h>
#include <controller_manager_msgs/SwitchController.h>
#include <geometry_msgs/Twist.h>

const std::string WALK_CMD_VEL_TOPIC = "/joy_vel";

int main(int argc, char **argv)
{

  ros::init(argc, argv, "walking_client_topic");
  ros::NodeHandle n;

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

  return 0;
}
