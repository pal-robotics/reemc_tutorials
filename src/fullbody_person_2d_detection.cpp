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
 * @brief example on how to get detections of fullbody standing persons in 2D images that
 *        are published by the /fullbody_2d_detector node.
 *
 *        This example will subscribe to the image topic /stereo/left/image and the
 *        topic fullbody_2d_detector/detections where the person detections in the metioned
 *        image are published. Then, it will show with rectangles the image regions in which
 *        persons have been detected.
 *
 * @author Jordi Pages.
 */

// PAL headers
#include <pal_detection_msgs/Detections2d.h>

// ROS headers
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

// OpenCV headers
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


/**
 * @brief callback callback function when a pair of synchronized image and person detections
 *        is found
 * @param imageMsg pointer to the image message
 * @param detectionsMsg poiner to the detections message
 */
void callback(const sensor_msgs::ImageConstPtr& imageMsg,
              const pal_detection_msgs::Detections2dConstPtr& detectionsMsg)
{
  // Get an OpenCV image from the image message
  cv_bridge::CvImageConstPtr cvImgPtr;
  cvImgPtr = cv_bridge::toCvShare(imageMsg); 
  cv::Mat img;
  cvImgPtr->image.copyTo(img);

  // Paint every detection on the image
  for (unsigned int i = 0; i < detectionsMsg->detections.size(); ++i)
  {
    //paint every detection as a rectangle
    cv::Rect roi;
    roi.x      = detectionsMsg->detections[i].x;
    roi.y      = detectionsMsg->detections[i].y;
    roi.width  = detectionsMsg->detections[i].width;
    roi.height = detectionsMsg->detections[i].height;
    cv::rectangle(img, roi, CV_RGB(0,255,255), 2);
  }

  // Show the image with the person detections
  cv::imshow("full body detections", img);
  cv::waitKey(15);

}

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "fullbody_person_2d_detection");

  ROS_INFO("Starting fullbody_person_2d_detection application ...");

  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(5.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }   

  // Define an image topic subscriber
  message_filters::Subscriber<sensor_msgs::Image> imageSub(nh, "stereo/left/image", 1);

  // Define a subscriber to the topic with the person detections
  message_filters::Subscriber<pal_detection_msgs::Detections2d> detectionsSub(nh, "fullbody_2d_detector/detections", 1);

  // Create a synchronizer to obtain pairs of images and person detections
  message_filters::TimeSynchronizer<sensor_msgs::Image,
                                    pal_detection_msgs::Detections2d> topicSynchronizer(imageSub, detectionsSub, 20);

  // Register a callback that will be called for every pair of synchronized image and detection messages
  topicSynchronizer.registerCallback(boost::bind(&callback, _1, _2));

  // Let ROS attend callbacks
  ros::spin();

  return EXIT_SUCCESS;
}
