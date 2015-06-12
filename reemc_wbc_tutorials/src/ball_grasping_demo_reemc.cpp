#include <Eigen/Dense>
#include <ros/ros.h>
#include <rbdl_dynamics/rbdlSimulator.h>
#include <rbdl/rbdl_utils.h>
#include <Eigen/Dense>
#include <walking_tools/reference/reference_abstract.h>

using namespace Eigen;

int main(int argc, char** argv){

  ros::init(argc, argv, "ball_grasping_demo");
  //ros::NodeHandle nh("~");
  ros::NodeHandle nh("~");

  double dt = 0.01;
  nh.setParam("dt", dt);

  ros::Publisher head_goal_pub =  nh.advertise<geometry_msgs::PoseStamped>("/whole_body_kinematic_controler/gaze_objective_stereo_optical_frame_goal", 10);
  ros::Publisher right_arm_goal = nh.advertise<geometry_msgs::PoseStamped>("/whole_body_kinematic_controler/arm_right_7_link_goal", 10);
  ros::Publisher left_arm_goal = nh.advertise<geometry_msgs::PoseStamped>("/whole_body_kinematic_controler/arm_left_7_link_goal", 10);

  Eigen::Matrix3d E = Eigen::Matrix3d::Identity();

  Eigen::Vector3d r_head(0.8, 0.0, 0.5);
  ReferenceAbstractPtr head_reference = ReferenceAbstract::create("interactive_marker", nh,
                                        "head_target", "/world", r_head, Eigen::Quaternion<double>(E));

  Eigen::Vector3d r_right(0.2, -0.2, 0.0);
  ReferenceAbstractPtr right_arm_reference = ReferenceAbstract::create("interactive_marker", nh,
                                        "right_arm_target", "/world", r_right, Eigen::Quaternion<double>(E));

  Eigen::Vector3d r_left(0.2, 0.2, 0.0);
  ReferenceAbstractPtr left_arm_reference = ReferenceAbstract::create("interactive_marker", nh,
                                        "left_arm_target", "/world", r_left, Eigen::Quaternion<double>(E));

  while(nh.ok()){
    ros::Time time = ros::Time::now();
    ros::spinOnce();

    head_reference->integrate(time, ros::Duration(0.0));
    right_arm_reference->integrate(time, ros::Duration(0.0));
    left_arm_reference->integrate(time, ros::Duration(0.0));

    /*
    if(head_reference->newData()){
      SignalReference new_target;
      head_reference->getTarget(new_target);

      geometry_msgs::PoseStamped pos;
      pos.header.stamp = time;
      pos.header.frame_id = "/world";
      pos.pose.position.x = new_target.targetPosition(0);
      pos.pose.position.y = new_target.targetPosition(1);
      pos.pose.position.z = new_target.targetPosition(2);

      pos.pose.orientation.x = new_target.targetOrientation.x();
      pos.pose.orientation.y = new_target.targetOrientation.y();
      pos.pose.orientation.z = new_target.targetOrientation.z();
      pos.pose.orientation.w = new_target.targetOrientation.w();
      head_goal_pub.publish(pos);

    }
    */

    // Only publish new goal if interactive marker has been updated, if not it chokes the controller interpolator
    if(right_arm_reference->newData()){
      SignalReference new_target;
      right_arm_reference->getTarget(new_target);

      geometry_msgs::PoseStamped pos;
      pos.header.stamp = time;
      pos.header.frame_id = "/world";
      pos.pose.position.x = new_target.targetPosition(0);
      pos.pose.position.y = new_target.targetPosition(1);
      pos.pose.position.z = new_target.targetPosition(2);

      pos.pose.orientation.x = new_target.targetOrientation.x();
      pos.pose.orientation.y = new_target.targetOrientation.y();
      pos.pose.orientation.z = new_target.targetOrientation.z();
      pos.pose.orientation.w = new_target.targetOrientation.w();
      right_arm_goal.publish(pos);
      head_goal_pub.publish(pos);
    }

    if(left_arm_reference->newData()){
      SignalReference new_target;
      left_arm_reference->getTarget(new_target);

      geometry_msgs::PoseStamped pos;
      pos.header.stamp = time;
      pos.header.frame_id = "/world";
      pos.pose.position.x = new_target.targetPosition(0);
      pos.pose.position.y = new_target.targetPosition(1);
      pos.pose.position.z = new_target.targetPosition(2);

      pos.pose.orientation.x = new_target.targetOrientation.x();
      pos.pose.orientation.y = new_target.targetOrientation.y();
      pos.pose.orientation.z = new_target.targetOrientation.z();
      pos.pose.orientation.w = new_target.targetOrientation.w();

      left_arm_goal.publish(pos);
    }

    ros::Duration(dt).sleep();
  }
}

