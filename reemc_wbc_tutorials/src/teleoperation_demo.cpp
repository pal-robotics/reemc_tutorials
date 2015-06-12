#include <Eigen/Dense>
#include <ros/ros.h>
#include <rbdl_dynamics/rbdlSimulator.h>
#include <rbdl/rbdl_utils.h>
#include <Eigen/Dense>
#include <walking_tools/walking_visualization_tools.h>
#include <walking_tools/conversions.h>

/*
 * 1. Get the trasnforms of the human with respect to its inertial frame_
 * 2. Get the transforms of the robot with respect to its inertial frame
 * 3. Compute the normalization factors to map
 *
 */

using namespace Eigen;

boost::shared_ptr <tf::TransformListener> listener;
ros::Publisher marker_pub;

enum frames_t {origin, left_shoulder, left_elbow,left_hand,
               right_shoulder, right_elbow, right_hand,
               torso};

std::string human_frames[] = {"openni_depth_frame", "left_shoulder", "left_elbow", "left_hand", "right_shoulder", "right_elbow", "right_hand", "torso"};


std::string robot_frames[] = {"world", "arm_left_1_link", "arm_left_4_link", "arm_left_7_link", "arm_left_1_link", "arm_left_4_link", "arm_left_7_link", "base_link"};

std::vector<eMatrixHom> kinect_map(7);
std::vector<eMatrixHom> robot_map(7);
VectorXd kinect_proportions(3);
VectorXd robot_proportions(3);
VectorXd proportions_rate(3);

int main(int argc, char** argv){

  ros::init(argc, argv, "reemc_teleoperation_demo");
  ros::NodeHandle nh;

  marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/teleoperation_markers", 10);

  ros::Publisher right_arm_goal = nh.advertise<geometry_msgs::PoseStamped>("/topic2", 10);
  ros::Publisher left_arm_goal = nh.advertise<geometry_msgs::PoseStamped>("/topic2", 10);
  ros::Publisher right_elbow_goal = nh.advertise<geometry_msgs::PoseStamped>("/topic2", 10);
  ros::Publisher left_elbow_goal = nh.advertise<geometry_msgs::PoseStamped>("/topic2", 10);
  ros::Publisher torso_goal = nh.advertise<geometry_msgs::PoseStamped>("/topic2", 10);

  double dt = 0.01;

  //Wait for kinect frames to appear
  ros::Time now = ros::Time::now();
  while(!listener->waitForTransform (human_frames[origin], human_frames[torso], now, ros::Duration (1.0), ros::Duration (0.01))){
    ROS_INFO_STREAM("Waiting for frames from the kinect");
    now = ros::Time::now();
  }

  try{

    kinect_map[origin].setIdentity();
    tf::StampedTransform stored_transform;

    listener->lookupTransform (human_frames[origin], human_frames[torso], ros::Time(0), stored_transform);
    pal::convert(stored_transform, kinect_map[torso]);

    //Left

    listener->lookupTransform (human_frames[torso], human_frames[left_shoulder], ros::Time(0), stored_transform);
    pal::convert(stored_transform, kinect_map[left_shoulder]);

    listener->lookupTransform (human_frames[left_shoulder], human_frames[left_elbow], ros::Time(0), stored_transform);
    pal::convert(stored_transform, kinect_map[left_elbow]);

    listener->lookupTransform (human_frames[left_elbow], human_frames[left_hand], ros::Time(0), stored_transform);
    pal::convert(stored_transform, kinect_map[left_hand]);

    //Right

    listener->lookupTransform (human_frames[torso], human_frames[right_shoulder], ros::Time(0), stored_transform);
    pal::convert(stored_transform, kinect_map[right_shoulder]);

    listener->lookupTransform (human_frames[right_shoulder], human_frames[right_elbow], ros::Time(0), stored_transform);
    pal::convert(stored_transform, kinect_map[right_elbow]);

    listener->lookupTransform (human_frames[right_elbow], human_frames[right_hand], ros::Time(0), stored_transform);
    pal::convert(stored_transform, kinect_map[right_hand]);

  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR ("%s", ex.what ());
    return 0;
  }

  try{
    robot_map[origin].setIdentity();
    tf::StampedTransform stored_transform;

    listener->lookupTransform (robot_frames[origin], robot_frames[torso], ros::Time(0), stored_transform);
    pal::convert(stored_transform, robot_map[torso]);

    //Left

    listener->lookupTransform (robot_frames[torso], robot_frames[left_shoulder], ros::Time(0), stored_transform);
    pal::convert(stored_transform, robot_map[left_shoulder]);

    listener->lookupTransform (robot_frames[left_shoulder], robot_frames[left_elbow], ros::Time(0), stored_transform);
    pal::convert(stored_transform, robot_map[left_elbow]);

    listener->lookupTransform (robot_frames[left_elbow], robot_frames[left_hand], ros::Time(0), stored_transform);
    pal::convert(stored_transform, robot_map[left_hand]);

    //Right

    listener->lookupTransform (robot_frames[torso], robot_frames[right_shoulder], ros::Time(0), stored_transform);
    pal::convert(stored_transform, robot_map[right_shoulder]);

    listener->lookupTransform (robot_frames[right_shoulder], robot_frames[right_elbow], ros::Time(0), stored_transform);
    pal::convert(stored_transform, robot_map[right_elbow]);

    listener->lookupTransform (robot_frames[right_elbow], robot_frames[right_hand], ros::Time(0), stored_transform);
    pal::convert(stored_transform, robot_map[right_hand]);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR ("%s", ex.what ());
    return 0;
  }

  //We assume that the robot and human are symetrical
  kinect_proportions(0) = (kinect_map[torso].translation() - kinect_map[left_shoulder].translation()).norm();
  kinect_proportions(1) = (kinect_map[left_shoulder].translation() - kinect_map[left_elbow].translation()).norm();
  kinect_proportions(2) = (kinect_map[left_elbow].translation() - kinect_map[left_hand].translation()).norm();

  robot_proportions(0) = (robot_map[torso].translation() - robot_map[left_shoulder].translation()).norm();
  robot_proportions(1) = (robot_map[left_shoulder].translation() - robot_map[left_elbow].translation()).norm();
  robot_proportions(2) = (robot_map[left_elbow].translation() - robot_map[left_hand].translation()).norm();

  for(unsigned int i=0; i<3; ++i){
    proportions_rate(i) = kinect_proportions(i)/robot_proportions(i);
  }

  eMatrixHom initial_robot_pose = robot_map[origin];
  eMatrixHom initial_human_pose = kinect_map[origin];

  while(nh.ok()){

    ros::spinOnce();
    try{
      kinect_map[origin].setIdentity();
      tf::StampedTransform stored_transform;

      listener->lookupTransform (human_frames[origin], human_frames[torso], ros::Time(0), stored_transform);
      pal::convert(stored_transform, kinect_map[torso]);

      //Left

      listener->lookupTransform (human_frames[torso], human_frames[left_shoulder], ros::Time(0), stored_transform);
      pal::convert(stored_transform, kinect_map[left_shoulder]);

      listener->lookupTransform (human_frames[left_shoulder], human_frames[left_elbow], ros::Time(0), stored_transform);
      pal::convert(stored_transform, kinect_map[left_elbow]);

      listener->lookupTransform (human_frames[left_elbow], human_frames[left_hand], ros::Time(0), stored_transform);
      pal::convert(stored_transform, kinect_map[left_hand]);

      //Right

      listener->lookupTransform (human_frames[torso], human_frames[right_shoulder], ros::Time(0), stored_transform);
      pal::convert(stored_transform, kinect_map[right_shoulder]);

      listener->lookupTransform (human_frames[right_shoulder], human_frames[right_elbow], ros::Time(0), stored_transform);
      pal::convert(stored_transform, kinect_map[right_elbow]);

      listener->lookupTransform (human_frames[right_elbow], human_frames[right_hand], ros::Time(0), stored_transform);
      pal::convert(stored_transform, kinect_map[right_hand]);

    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR ("%s", ex.what ());
      return 0;
    }

    std::vector<eMatrixHom> retargetted_skeleton(7);
    //initialT^R * initialT^H^-1 * T^H
    retargetted_skeleton[0] = initial_robot_pose*initial_human_pose.inverse()*kinect_map[torso];

    //Left arm retargetting
    kinect_map[1].translation() = (kinect_map[left_shoulder].translation().normalized())*proportions_rate(0);
    retargetted_skeleton[1] = retargetted_skeleton[0]*kinect_map[1];

    kinect_map[2].translation() = (kinect_map[left_elbow].translation().normalized())*proportions_rate(1);
    retargetted_skeleton[2] = retargetted_skeleton[1]*kinect_map[2];

    kinect_map[3].translation() = kinect_map[left_hand].translation().normalized()*proportions_rate(2);
    retargetted_skeleton[3] = retargetted_skeleton[2]*kinect_map[3];

    //Right arm retargetting
    kinect_map[4].translation() = kinect_map[right_shoulder].translation().normalized()*proportions_rate(0);
    retargetted_skeleton[4] = retargetted_skeleton[0]*kinect_map[4];

    kinect_map[5].translation() = kinect_map[right_elbow].translation().normalized()*proportions_rate(1);
    retargetted_skeleton[5] = retargetted_skeleton[4]*kinect_map[5];

    kinect_map[6].translation() = kinect_map[right_hand].translation().normalized()*proportions_rate(2);
    retargetted_skeleton[6] = retargetted_skeleton[5]*kinect_map[6];

    //Draw
    visualization_msgs::MarkerArray marray;
    marray.markers.resize(7);
    unsigned int index = 0;
    ros::Time time = ros::Time::now();
    Eigen::Vector3d red; red<<1,0,0;
    for(unsigned int i=0; i<retargetted_skeleton.size(); ++i){
      eVector3 r = retargetted_skeleton[i].translation();
      publish_sphere(r, red, 0.05, "world", "retarget", time, marray, index);
    }

    marker_pub.publish(marray);

    ros::Duration(dt).sleep();
  }
}
