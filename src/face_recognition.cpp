/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
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

/** @file
 *
 * @brief example on how to get notifications of recognized faces from the ROS node pal_face
 *
 *        In order to get recognized persons first run face_enrollment in order to enroll persons
 *        in the test face database.
 *
 * @author Jordi Pages.
 */

// PAL headers
#include <pal_face_database/database.h>
#include <pal_detection_msgs/SetDatabase.h>
#include <pal_detection_msgs/Recognizer.h>

// ROS headers
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// OpenCV headers
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


/**
 * @brief callback callback function when a pair of synchronized image and person detections
 *        is found. The given face detected are
 * @param imageMsg pointer to the image message
 * @param detectionsMsg poiner to the detections message
 */
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // Get an OpenCV image from the image message
  cv_bridge::CvImageConstPtr cvImgPtr;
  cvImgPtr = cv_bridge::toCvShare(msg);
  cv::Mat img;
  cvImgPtr->image.copyTo(img);

  cv::namedWindow("face node debug", cv::WINDOW_NORMAL);
  cv::imshow("face node debug", img);
  cv::waitKey(15);
}

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "face_recognition");

  ROS_INFO("Starting face_recognition application ...");

  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(5.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }   


  // Subscribe to the debug image published by the node pal_face in which all face detections are painted
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber imageSub = it.subscribe("pal_face/debug", 1, imageCallback);


  // Set an empty face database for the test which will be stored in the test path
  std::string databaseName = "tutorial_database";

  // Use the ROS service in pal_face to set the database
  ros::ServiceClient setDatabaseClient = nh.serviceClient<pal_detection_msgs::SetDatabase>("pal_face/set_database");

  pal_detection_msgs::SetDatabase setDatabaseSrv;
  setDatabaseSrv.request.databaseName = databaseName;
  if (setDatabaseClient.call(setDatabaseSrv))
    ROS_INFO_STREAM("Face database succesffully created at: " << databaseName);
  else
  {
    ROS_ERROR_STREAM("Failure while creating face database at: " << databaseName);
    return EXIT_FAILURE;
  }


  // Enable face recognition in the pal_face
  ros::ServiceClient recognitionClient = nh.serviceClient<pal_detection_msgs::Recognizer>("pal_face/recognizer");
  pal_detection_msgs::Recognizer recognizerSrv;
  recognizerSrv.request.enabled = true;
  // Choose the minimum recognition confidence to publish recognized faces
  recognizerSrv.request.minConfidence = 80;
  if (recognitionClient.call(recognizerSrv))
    ROS_INFO_STREAM("Face recognition enabled with a minimum confidence of " << recognizerSrv.request.minConfidence);
  else
  {
    ROS_ERROR_STREAM("Unable to enable face recognition");
    return EXIT_FAILURE;
  }

  ROS_INFO("Press CTRL+C in this terminal to exit the application");

  ros::spin();

  return EXIT_SUCCESS;
}
