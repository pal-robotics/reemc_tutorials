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
 * @brief example on how to enroll a new person in a database so that the corresponding ROS node
 *        may notify when the given person is recognized in the left camera image.
 *
 *        First of all, the application asks the user to enter the name of the person to enroll. Then,
 *        the person is asked to stand in front of the robot in order to gather some samples of his/her
 *        face. When the user presses CTRL+C the application stops gathering face samples and proceeds
 *        to enroll the faces with the given name.
 *
 *        Note that a single person must be in front of the robot during the enrollment.
 *
 * @author Jordi Pages.
 */

// PAL headers
#include <pal_detection_msgs/SetDatabase.h>
#include <pal_detection_msgs/StartEnrollment.h>
#include <pal_detection_msgs/StopEnrollment.h>
#include <pal_face_database/database.h>

// ROS headers
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// OpenCV headers
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// Std C++ headers
#include <iostream>


/**
 * @brief callback callback function for the debug image published by the /pal_face ROS node in which
 *        face detections are already painted.
 * @param imageMsg pointer to the image message
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
  ros::init(argc, argv, "face_enrollment");

  ROS_INFO("Starting face_enrollment application ...");

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
  //empty the database
  setDatabaseSrv.request.purgeAll     = true;
  if (setDatabaseClient.call(setDatabaseSrv))
    ROS_INFO_STREAM("Face database succesffully created at: " << databaseName);
  else
  {
    ROS_ERROR_STREAM("Failure calling service pal_face/set_database. Is pal_face running?");
    return EXIT_FAILURE;
  }


  // Ask for person name
  std::cout << std::endl << "Enter the name of the person to enroll and press enter: " << std::endl;
  std::string name;
  getline(std::cin, name, '\n');


  // Start enrollment process with the appropriate ROS service
  ros::ServiceClient startEnrollmentClient = nh.serviceClient<pal_detection_msgs::StartEnrollment>("pal_face/start_enrollment");
  pal_detection_msgs::StartEnrollment startEnrollmentSrv;
  startEnrollmentSrv.request.name = name;
  if (startEnrollmentClient.call(startEnrollmentSrv))
    ROS_INFO_STREAM("Face enrollment for person " << name << " has started. Gathering faces until a key is pressed in the image window ...");
  else
  {
    ROS_ERROR_STREAM("Failure when starting person enrollment");
    return EXIT_FAILURE;
  }

  // Wait until a key is pressed
  bool keyPressed = false;
  while ( ros::ok() && !keyPressed)
  {
    //Let ROS attend callbacks
    ros::spinOnce();
    keyPressed = cv::waitKey(50) != -1;
  }

  ROS_INFO("Key pressed. Proceeding to complete the enrollment...");

  // Stop enrollment
  ros::ServiceClient stopEnrollmentClient = nh.serviceClient<pal_detection_msgs::StopEnrollment>("pal_face/stop_enrollment");
  pal_detection_msgs::StopEnrollment stopEnrollmentSrv;
  if ( stopEnrollmentClient.call(stopEnrollmentSrv) )
  {
    ROS_INFO_STREAM("Face enrollment for person " << name <<
                    " ended. A total of " << stopEnrollmentSrv.response.numFacesEnrolled <<
                    " faces have been gathered");
  }
  else
  {
    ROS_ERROR_STREAM("Failure when stoping the person enrollment: " << stopEnrollmentSrv.response.error_msg);
    return EXIT_FAILURE;
  }

  ROS_INFO(" ");

  return EXIT_SUCCESS;
}
