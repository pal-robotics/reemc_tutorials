/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2014, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Jordi Pages. */

/**
 * @file
 *
 * @brief example on how to subscribe to an image topic and how to make the robot look towards a textured object
 *
 *
 * How to test this application in simulation:
 *
 * 1) Launch a simulation of REEM-C
 *
 *   $ roslaunch reemc_tutorials reemc_look_to_object_world.launch
 *
 * 2) Open a new terminal and launch the head controllers:
 *   $ roslaunch reemc_controller_configuration joint_trajectory_controllers.launch
 *
 * 3) Run in another terminal the object detector:
 *
 *   $ roslaunch pal_texture_detector_node texture_detector.launch
 *
 * 4) In a new terminal launch the application:
 *
 *   $ rosrun reemc_tutorials look_to_object
 *
 * 5) Select the object in gazebo and move it or turn it and see how REEM-C keeps looking at it
 *
 *
 * How to test this application with the actual robot:
 *
 * 1) In a terminal run
 *
 *   1.1) $ ssh -X pal@reemc-1m
 *   1.2) $ textureDetectorStart.sh
 *
 * 2) In a new terminal run
 *
 *   4.1) $ ssh -X pal@reemc-1m
 *   4.2) $ roslaunch reemc_controller_configuration joint_trajectory_controllers.launch
 *
 * 3) rosrun reemc_tutorials look_to_object
 *
 * 5) Move the PAL textured object in front of REEM-C so that it trakcs the object with the gaze
 *
 */

// PAL heades
#include <pal_detection_msgs/TexturedObjectDetection.h>

// ROS headers
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>
#include <control_msgs/PointHeadAction.h>

// OpenCV headers
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// Boost headers
#include <boost/shared_ptr.hpp>

// C++ standard headers
#include <exception>
#include <string>


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const std::string windowName         = "object detection";
static const std::string cameraFrame        = "/stereo_optical_frame";
static const std::string cameraInfoTopic    = "stereo/right/camera_info";
static const std::string detectorImageTopic = "texture_detector/debug";
static const std::string detectionTopic     = "texture_detector/detection";


// Intrinsic parameters of the camera
cv::Mat cameraIntrinsics;
bool intrinsicsReceived;

// Our Action interface type for moving REEM-C's head, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> PointHeadClient;
typedef boost::shared_ptr<PointHeadClient> PointHeadClientPtr;

PointHeadClientPtr pointHeadClient;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ROS call back for every new image received
void detectionImageCallback(const sensor_msgs::ImageConstPtr& imgMsg)
{
  cv_bridge::CvImagePtr cvImgPtr;

  cvImgPtr = cv_bridge::toCvCopy(imgMsg, imgMsg->encoding);
  cv::imshow(windowName, cvImgPtr->image);
  cv::waitKey(15);
}


// ROS call back for every object detection
void detectionCallback(const pal_detection_msgs::TexturedObjectDetectionConstPtr& msg)
{
  //first check if the object has been detected
  if ( !msg->roi.x.empty() )
  {
    //compute the centroid of the detection roi in the image
    int u = 0, v = 0;
    for (unsigned int i = 0; i < msg->roi.x.size(); ++i)
    {
      u += msg->roi.x[i];
      v += msg->roi.y[i];
    }

    u = u / msg->roi.x.size();
    v = v / msg->roi.y.size();

    geometry_msgs::PointStamped pointStamped;

    pointStamped.header.frame_id = cameraFrame;
    pointStamped.header.stamp    = ros::Time::now();

    //compute normalized coordinates of the selected pixel
    double x = ( u  - cameraIntrinsics.at<double>(0,2) )/ cameraIntrinsics.at<double>(0,0);
    double y = ( v  - cameraIntrinsics.at<double>(1,2) )/ cameraIntrinsics.at<double>(1,1);
    double Z = 1.0; //define an arbitrary distance
    pointStamped.point.x = x * Z;
    pointStamped.point.y = y * Z;
    pointStamped.point.z = Z;

    //build the action goal
    control_msgs::PointHeadGoal goal;
    //the goal consists in making the Z axis of the cameraFrame to point towards the pointStamped
    goal.pointing_frame = cameraFrame;
    goal.pointing_axis.x = 0.0;
    goal.pointing_axis.y = 0.0;
    goal.pointing_axis.z = 1.0;
    goal.min_duration = ros::Duration(1.0);
    goal.max_velocity = 0.25;
    goal.target = pointStamped;

    pointHeadClient->sendGoal(goal);
    ros::Duration(0.5).sleep();
  }
}

// ROS callback function for topic containing intrinsic parameters of a camera
void getCameraIntrinsics(const sensor_msgs::CameraInfoConstPtr& msg)
{
  cameraIntrinsics = cv::Mat::zeros(3,3,CV_64F);

  cameraIntrinsics.at<double>(0, 0) = msg->K[0]; //fx
  cameraIntrinsics.at<double>(1, 1) = msg->K[4]; //fy
  cameraIntrinsics.at<double>(0, 2) = msg->K[2]; //cx
  cameraIntrinsics.at<double>(1, 2) = msg->K[5]; //cy
  cameraIntrinsics.at<double>(2, 2) = 1;
}

// Create a ROS action client to move REEM-C's head
void createPointHeadClient(PointHeadClientPtr& actionClient)
{
  ROS_INFO("Creating action client to head controller ...");

  actionClient.reset( new PointHeadClient("/head_controller/point_head_action") );

  int iterations = 0, max_iterations = 3;
  // Wait for head controller action server to come up
  while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the point_head_action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createPointHeadClient: head controller action server not available");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "look_to_point");

  ROS_INFO("Starting look_to_point application ...");

  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(5.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  ROS_INFO("Waiting for camera intrinsics ... ");

  // Get the camera intrinsic parameters from the appropriate ROS topic
  sensor_msgs::CameraInfoConstPtr camInfoMsg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cameraInfoTopic, nh);
  getCameraIntrinsics(camInfoMsg);

  // Create a point head action client to move the REEM-C's head
  createPointHeadClient( pointHeadClient );

  // Create the window to show REEM-C's camera images
  cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);

  // Define ROS topic from where REEM-C publishes images
  image_transport::ImageTransport it(nh);
  image_transport::TransportHints transportHint("compressed");

  ROS_INFO_STREAM("Subscribing to " << detectorImageTopic << " ...");
  image_transport::Subscriber sub = it.subscribe(detectorImageTopic, 1,
                                                 detectionImageCallback, transportHint);

  ROS_INFO_STREAM("Subscribing to " << detectionTopic << " ...");
  ros::Subscriber detectionSub = nh.subscribe(detectionTopic, 1, detectionCallback);

  //enter a loop that processes ROS callbacks. Press CTRL+C to exit the loop
  ros::spin();

  cv::destroyWindow(windowName);

  return EXIT_SUCCESS;
}
