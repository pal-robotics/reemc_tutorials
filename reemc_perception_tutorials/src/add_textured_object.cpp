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

// PAL headers
#include <pal_detection_msgs/AddTexturedObject.h>

// ROS headers
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// OpenCV headers
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// Boost headers
#include <boost/shared_ptr.hpp>

// C++ standard headers
#include <exception>
#include <string>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const std::string windowName      = "REEM-C left eye";
static const std::string cameraFrame     = "/stereo_optical_frame";
static const std::string imageTopic      = "stereo/left/image";

enum status {
              WAITING_OBJECT_NAME = 0,
              SELECTING_SNAPSHOT = 1,
              WAITING_TOPLEFT_CORNER = 2,
              WAITING_BOTTOMRIGHT_CORNER = 3,
              ROI_DONE = 4
            };

status currentStatus;

cv::Point topLeftCorner, bottomRightCorner;
cv::Mat img;
cv::Rect objectRoi;

ros::CallbackQueue cbQueue;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ROS call back for every new image received
void imageCallback(const sensor_msgs::ImageConstPtr& imgMsg)
{
  cv_bridge::CvImagePtr cvImgPtr;

  cvImgPtr = cv_bridge::toCvCopy(imgMsg, imgMsg->encoding);
  img = cvImgPtr->image.clone();

  cv::imshow(windowName, img);
  cv::waitKey(15);
}

// OpenCV callback function for mouse events on a window
void onMouse( int event, int u, int v, int, void* )
{
  if ( event != cv::EVENT_LBUTTONDOWN )
      return;

  if ( currentStatus == SELECTING_SNAPSHOT )
  {
    currentStatus = WAITING_TOPLEFT_CORNER;
  }
  else if ( currentStatus == WAITING_TOPLEFT_CORNER )
  {
    topLeftCorner = cv::Point(u, v);    
    currentStatus = WAITING_BOTTOMRIGHT_CORNER;
  }
  else if ( currentStatus == WAITING_BOTTOMRIGHT_CORNER )
  {
    bottomRightCorner = cv::Point(u, v);
    currentStatus = ROI_DONE;
  }
}

void selectObjectSnapshot()
{
  std::cout << "Click in the image to take a suitable snapshot of the object" << std::endl;
  double period = 1/15;
  ros::Rate loopRate(1.0/period);

  while ( ros::ok() && currentStatus == SELECTING_SNAPSHOT )
  {
    cbQueue.callAvailable(ros::WallDuration(period));
    loopRate.sleep();
  }
}

bool selectObjectRoi()
{
  std::cout << std::endl << "Click on the top-left corner of the object" << std::endl;

  //wait for top-left corner
  while ( currentStatus == WAITING_TOPLEFT_CORNER && ros::ok() )
  {
    cv::waitKey(50);
  }

  std::cout << std::endl << "Click on the bottom-right corner of the object" << std::endl;

  //wait for bottom-right corner
  while ( currentStatus == WAITING_BOTTOMRIGHT_CORNER && ros::ok() )
  {
    cv::waitKey(50);
  }

  //draw the rectangular roi based on the two selected corners
  objectRoi.x = topLeftCorner.x;
  objectRoi.y = topLeftCorner.y;
  objectRoi.width  = bottomRightCorner.x - topLeftCorner.x + 1;
  objectRoi.height = bottomRightCorner.y - topLeftCorner.y + 1;
  cv::Mat imgWithRoi = img.clone();
  cv::rectangle(imgWithRoi, objectRoi, CV_RGB(255,0,0), 2);
  cv::imshow(windowName, imgWithRoi);
  cv::waitKey(15);

  std::cout << "Is the ROI good enough? Type Y or N: " << std::endl;
  std::string response;
  getline(std::cin, response, '\n');
  if ( response == "Y" || response == "y" )
    return true;
  else
    return false;
}

void imgToMsg(const cv::Mat& inputImg,
              sensor_msgs::Image& imgMsg)
{
  cv_bridge::CvImage cvImg;
  cvImg.encoding = sensor_msgs::image_encodings::BGR8;

  cvImg.image = inputImg.clone();
  cvImg.toImageMsg(imgMsg);
}

int addNewObject(ros::NodeHandle& nh,
                 const std::string& objectName)
{
  std::cout << "Enter the full path where the object image will be stored: " << std::endl;
  std::string fullPathImage;
  getline(std::cin, fullPathImage, '\n');
  if ( fullPathImage.empty() )
    throw std::runtime_error("Error in addNewObject: empty path is not valid");

  //add ending slash if necessary
  if ( fullPathImage.substr(fullPathImage.size()-1, 1) != "/" )
    fullPathImage += "/";

  std::string serviceName("texture_detector/add_object");

  ros::ServiceClient addObjectClient = nh.serviceClient<pal_detection_msgs::AddTexturedObject>(serviceName);
  pal_detection_msgs::AddTexturedObject addObjectSrv;
  imgToMsg(img(objectRoi), addObjectSrv.request.img);
  addObjectSrv.request.fullPathFileName = fullPathImage + objectName;

  if (addObjectClient.call(addObjectSrv))
  {
    if ( addObjectSrv.response.result == true )
      std::cout << "Object image stored correctly in: " << addObjectSrv.request.fullPathFileName << ".png" << std::endl
                << " (in the computer where the texture_detector node is running)" << std::endl << std::endl;
    else
    {
      std::cout << "Object image could not be stored in: " << addObjectSrv.request.fullPathFileName << std::endl;
      std::cout << "Check that the path exists in the target computer" << std::endl << std::endl;
      return EXIT_FAILURE;
    }
  }
  else
  {
    std::cout <<"Failure calling service " << serviceName << ". Make sure that texture_detector node is running";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "add_textured_object");

  std::cout << "Starting add_textured_object application ..." << std::endl;
 
  // Precondition: Valid cloc
  ros::NodeHandle nh;  
  nh.setCallbackQueue(&cbQueue);

  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  currentStatus = WAITING_OBJECT_NAME;

  // Ask for object's name
  std::cout << std::endl << "Enter the name of the object that will be learnt: " << std::endl;
  std::string objectName;
  getline(std::cin, objectName, '\n');    

  currentStatus = SELECTING_SNAPSHOT;

  // Create the window to show REEM-C's camera images
  cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);

  // Set mouse handler for the window
  cv::setMouseCallback(windowName, onMouse);

  // Define ROS topic from where REEM-C publishes images
  image_transport::ImageTransport it(nh);  
  // use compressed image transport to prevent delays
  image_transport::TransportHints transportHint("compressed");

  std::cout << "Subscribing to " << imageTopic << " ..." << std::endl << std::endl;
  image_transport::Subscriber sub = it.subscribe(imageTopic, 1,
                                                 imageCallback, transportHint);

  //wait user's selecting one of the images
  selectObjectSnapshot();

  //stop receiving new images
  sub.shutdown();

  //wait user's fitting a roi to the object appearing in the image
  bool roiOk = selectObjectRoi();

  int response = EXIT_SUCCESS;

  //store new object
  if ( roiOk )
    response = addNewObject(nh, objectName);

  cv::destroyWindow(windowName);

  return response;
}
