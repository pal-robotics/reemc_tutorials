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
 * @brief example on how to get images on demand from the stereo camera.
 *        This example works with any stereo camera published in ROS fashion by
 *        changing the two image and camera_info topic names.
 *
 * @author Jordi Pages.
 */

// PAL headers
#include <pal_camera_client/stereoCameraClient.h>
#include <pal_camera_client/utils.h>

// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

// OpenCV headers
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// Print on scren the intrinsic parameters encapsulated in sensors_msgs::CameraInfo
void printCalibrationData(const std::string& eye, const sensor_msgs::CameraInfo& camInfo)
{
  ROS_INFO(" ");
  ROS_INFO_STREAM("Calibration data of the " << eye << " camera:");
  ROS_INFO(" ");
  ROS_INFO_STREAM("width:     " << pal::cameraInfo::getImageWidth(camInfo) << " pixels");
  ROS_INFO_STREAM("height:    " << pal::cameraInfo::getImageHeight(camInfo) << " pixels");
  ROS_INFO_STREAM("fx:        " << pal::cameraInfo::getFx(camInfo) << " pixels");
  ROS_INFO_STREAM("fy:        " << pal::cameraInfo::getFy(camInfo) << " pixels");
  ROS_INFO_STREAM("cx:        " << pal::cameraInfo::getCx(camInfo) << " pixels");
  ROS_INFO_STREAM("cy:        " << pal::cameraInfo::getCy(camInfo) << " pixels");
  double k1, k2, k3, p1, p2;
  k1 = pal::cameraInfo::getDistortion(camInfo)[0];
  k2 = pal::cameraInfo::getDistortion(camInfo)[1];
  p1 = pal::cameraInfo::getDistortion(camInfo)[2];
  p2 = pal::cameraInfo::getDistortion(camInfo)[3];
  k3 = pal::cameraInfo::getDistortion(camInfo)[4];
  ROS_INFO_STREAM("distortion (k1, k2, k3, p1, p2) = (" << k1 << ", " << k2 << ", " << k3 << ", " << p1 << ", " << p2 << ")");
  ROS_INFO(" ");
}

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "remote_stereo_image_visualization");

  ROS_INFO("Starting remote_stereo_image_visualization application ...");

  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(5.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  double frequency = 5; //frequency at which images will be polled in this example

  // create a camera client able to provide images and calibration data on demand
  pal::StereoCameraClient camClient("stereo/left/image",                  //left image topic name
                                    "stereo/right/image",                 //right image topic name
                                    pal::StereoCameraClient::JPEG,         //image transport type. Choose between RAW (uncompressed) and JPEG
                                    pal::StereoCameraClient::APPROX_TIME,  //topics synchronization policy
                                    5.0,                                   //timeout in seconds attempting to get data from topics
                                    static_cast<float>(frequency*2),       //max rate at which images may be polled
                                    "stereo/left/camera_info",            //left camera info topic name
                                    "stereo/right/camera_info");          //right camera info topic name

  //print calibration data once
  sensor_msgs::CameraInfo camInfo_left, camInfo_right;
  camClient.getCameraInfos(camInfo_left, camInfo_right);

  printCalibrationData("left", camInfo_left);
  printCalibrationData("right", camInfo_right);

  //get images periodically and show them in a window
  cv::Mat img_left, img_right;
  cv::namedWindow("left image");
  cv::namedWindow("right image");

  ros::Rate rate(frequency);
  while ( ros::ok() )   //keep looping until CTRL+C is pressed
  {
    camClient.getImages(img_left, img_right);
    cv::imshow("left image", img_left);
    cv::imshow("right image", img_right);
    cv::waitKey(15);
    rate.sleep();
  }

  return EXIT_SUCCESS;
}
