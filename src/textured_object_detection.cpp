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
 * @brief example on how to get detections of the PAL Textured Object published when it is
 *        shown to REEM-C close, which are published by the texture_detector node.
 *
 *        This example will subscribe to the image topic stereo/left/image and the
 *        topic texture_detector/detection to get synchronized pairs.
 *        Then, it will show with rectangles the image region where the PAL Textured Object
 *        is detected when so.
 *
 * @author Jordi Pages.
 */

// PAL headers
#include <pal_detection_msgs/TexturedObjectDetection.h>

// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

// OpenCV headers
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


/**
 * @brief callback callback function when a pair of synchronized image and object detection
 *        is found
 * @param imageMsg pointer to the image message
 * @param detectionsMsg poiner to the detections message
 */
void callback(const pal_detection_msgs::TexturedObjectDetectionConstPtr& detectionMsg)
{  
  // Get an OpenCV image from the compressed image included in the message
  cv::Mat img;
  img = cv::imdecode(cv::Mat(detectionMsg->img.data), CV_LOAD_IMAGE_UNCHANGED);

  if ( !detectionMsg->roi.x.empty() )
  {
    std::vector< cv::Point > points;
    for (unsigned int i = 0; i < 4; ++i)
      points.push_back( cv::Point(detectionMsg->roi.x[i], detectionMsg->roi.y[i]) );

    ROS_INFO("Object detected!");

    cv::line(img, points[0], points[1], CV_RGB(0,255,0), 2);
    cv::line(img, points[1], points[2], CV_RGB(0,255,0), 2);
    cv::line(img, points[2], points[3], CV_RGB(0,255,0), 2);
    cv::line(img, points[0], points[3], CV_RGB(0,255,0), 2);
  }

  // Show the image with the person detections
  cv::imshow("Object detection", img);
  cv::waitKey(50);

}

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "textured_object_detection");

  ROS_INFO("Starting textured_object_detection application ...");

  // Precondition: Valid clock  
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(5.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }   

  ROS_INFO("Ok");

  ros::Subscriber sub = nh.subscribe("texture_detector/detection", 1, callback);

  ROS_INFO("Put the PAL Textured Object in front of the robot camera at about 1 m ...");

  // Let ROS attend callbacks
  ros::spin();

  return EXIT_SUCCESS;
}
