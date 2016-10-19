#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "img_service/TagDetection.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include "apriltags_ros/AprilTagDetectionArray.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <image_transport/image_transport.h>



const double PI = 3.1415;
const int SCAN_COUNT = 5;
cv::Mat tapes;

image_transport::Publisher pub;

bool lock = false;
sensor_msgs::Image::ConstPtr recent_img;
apriltags_ros::AprilTagDetectionArray::ConstPtr tags;

cv::Point2f tag_location(int tag_id)
{
  
  if (tags == NULL)
     {
  //TODO error?
  return cv::Point2f(0,0);
  }
  else 
  {
  int index;
  for (index=0; tags->detections[index].id != tag_id; index++)
  {}
  geometry_msgs::Pose pos = tags->detections[index].pose.pose;
  return cv::Point2f(pos.position.x, pos.position.y);
  //NOTE: theta = pos.orientation
  //return cv::Point2f(20,20);
  }
}

void publish_image(cv::Mat tapes_l)
{
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", tapes_l).toImageMsg();
  pub.publish(msg);
  ros::spinOnce();
}
void test_image(std::string path) 
{
  /*
     lower_blue = 101, 99, 8
     upper_blue = 125, 255, 112
     from experimentation
   */
  cv::Mat image = cv::imread(path, CV_LOAD_IMAGE_COLOR);
  if (image.empty()) 
  {
    ROS_INFO("Cant open image");
    return;
  }
  if (tapes.empty()) //init
  {
    tapes = cv::Mat(image.size(), CV_8UC1);
    tapes = cv::Scalar(127);
  }
  cv::blur(image,image, cv::Size(3,3)); //TODO might need to adjust
  cv::Mat img_hsv, img_threshold, img_display;
  cv::cvtColor(image, img_hsv, CV_BGR2HSV);
  cv::inRange(img_hsv, cv::Scalar(101,99,8), cv::Scalar(125,255,112) , img_display);
  //imshow(path+"_hsv", img_display);
  //std::vector<cv::Mat> channels;
  //  cv::split(img_hsv, channels);

  //  cv::threshold(img_hsv, img_threshold, 100.0f, 120.0f, 3);  
  //  cv::Mat d;
  //  cv::cvtColor(img_display, d, CV_HSV2BGR);

  cv::Point2f src = tag_location(10);
  std::vector<cv::Vec2f> lines;
  cv::Mat dst, cdst;  
  //  Canny(img_display, dst, 50, 200, 3);
  dst = img_display.clone();
  Canny(img_display, cdst, 50, 200, 3); 

  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(cdst, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point(0, 0) );
  cv::Mat drawing = cv::Mat::zeros(cdst.size(), CV_8UC3 );

  /* //probably don't need because it's taken care of later
     for (int i=1; i<contours.size()-1; i++)
     {
     if (cv::contourArea(contours[i], true) < 0) 
     {
  //remove duplicates
  contours.erase(contours.begin() + i--);
  }
  }*/

  int before = contours.size();
  for (int i=0; i<contours.size(); i++)
  {
    //    std::cout << cv::contourArea(contours[i]) << ", ";
    if (cv::contourArea(contours[i]) > 30) //TODO threshold
    {
      hierarchy.erase(hierarchy.begin() + i);
      contours.erase(contours.begin() + i--);
    }
  }
  std::cout << "Passed contours: " << before - contours.size() << "\n";

  /*
     for(int i=0; i<contours.size(); i++)
     {
     std::stringstream ss;
     ss << "contourr " << i;
     cv::Mat contour1 = cv::Mat::zeros(cdst.size(), CV_8UC3);
     cv::drawContours(contour1, contours, i, cv::Scalar(255,255,255),2,8,hierarchy,0,cv::Point() );
     imshow(ss.str(), contour1);
     }
   */

  std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
  std::vector<cv::Rect> boundRect( contours.size() );
  std::vector<cv::Point2f> center( contours.size() );
  std::vector<float>radius( contours.size() );
  for (int i=0; i<contours.size(); i++)
  {
    if (cv::contourArea(contours[i], true) < 0) //fix counter clockwise
    {
      std::reverse(contours[i].begin(), contours[i].end());
    }
  }
  before = contours.size();
  for (int i=0; i<contours.size(); i++)
  {
    cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
    boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
    cv::minEnclosingCircle( (cv::Mat)contours_poly[i], center[i], radius[i] );
    double tape_circle = 0.5; //TODO threshold
    double rect_obj = (cv::contourArea(contours[i]) / (boundRect[i].width*boundRect[i].height));
  //  std::cout << rect_obj << ", ";
    if (rect_obj > .8 && rect_obj < 1 && cv::contourArea(contours[i]) > 10)
    {
      center.erase(center.begin() + i);
      radius.erase(radius.begin() + i);
      boundRect.erase(boundRect.begin() + i);  
      hierarchy.erase(hierarchy.begin() + i);
      contours.erase(contours.begin() + i--);

    }
    /*    if (cv::contourArea(contours[i])/(PI*radius[i]*radius[i]) < tape_circle)
	  {
    //remove this, probably not tape
    //also remove these if plan to use after
    center.erase(center.begin() + i);
    radius.erase(radius.begin() + i);
    boundRect.erase(boundRect.begin() + i);  
    hierarchy.erase(hierarchy.begin() + i);
    contours.erase(contours.begin() + i--);
    }*/
  }
  std::cout << "Passed contours: " << before - contours.size() << "\n";
  for(int i = 0; i< contours.size(); i++ )
  {
    std::stringstream ss;
    ss << i;
    if (radius[i] < 1) radius[i] = 1.0f;
    radius[i] += 1.5;
    //    cv::drawContours( dst, contours, i, cv::Scalar(0), -1, 8, hierarchy, 0, cv::Point() );
    //  cv::drawContours(drawing, contours, i, cv::Scalar(255,255,255),-1,8,hierarchy,0,cv::Point());
    cv::circle(dst, center[i], radius[i], cv::Scalar(0),-1);
    cv::circle(drawing, center[i], radius[i], cv::Scalar(255,255,255),-1);
    cv::putText(drawing,ss.str() , center[i], cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1,
	cv::Scalar(255,0,0), 3, 3);
  }
  std::cout << "contour count: " << contours.size() << "\n";

  //  imshow(path, dst);

  //imshow("canny", dst);
  //  imshow(path+"_contour", drawing);
  /*
     cv::waitKey(0);*/
  cv::Mat tapes_copy = tapes.clone();
  cv::addWeighted(dst, 0.5, tapes_copy, 0.5, 0.0, tapes);
  return;
}


bool detect(img_service::TagDetection::Request &req, 
    img_service::TagDetection::Response &res)
{
  //  req.tag_Id;
  //  res.scan;
  lock = true;
  if (tapes.empty())
  {
    ROS_INFO("No image found");
    return false;
  }
  cv::Mat tapes_cpy = tapes.clone();
  lock = false;
  res.scan.angle_min = 0.0f;
  res.scan.angle_max = 2.0f*PI;
  res.scan.angle_increment = 2.0f*PI/SCAN_COUNT;
  res.scan.range_min = 0.0f;
  res.scan.range_max = 1000.0f; //TODO get max based on camera dimension?
  for (int i=0; i<SCAN_COUNT; i++)
  {
    res.scan.ranges[i] = 0.0f; //TODO fill
  }
  lock = false;
  return true;
}

void Callback_Img(const sensor_msgs::Image::ConstPtr& msg)
{

  if (lock) 
return;
  /*
     lower_blue = 101, 99, 8
     upper_blue = 125, 255, 112
     from experimentation
   */
//  cv::Mat image = cv::imread(path, CV_LOAD_IMAGE_COLOR);
  if (msg == NULL)
  {
    ROS_INFO("No image found");
    return;
  }
  cv_bridge::CvImagePtr cv_ptr;
  try 
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  

  cv::Mat image =  cv_ptr->image;

  if (image.empty()) 
  {
    ROS_INFO("Cant open image");
    return;
  }
  if (tapes.empty()) //init
  {
    tapes = cv::Mat(image.size(), CV_8UC1);
    tapes = cv::Scalar(127);
  }
  cv::blur(image,image, cv::Size(3,3)); //TODO might need to adjust
  cv::Mat img_hsv, img_threshold, img_display;
  cv::cvtColor(image, img_hsv, CV_BGR2HSV);
  cv::inRange(img_hsv, cv::Scalar(101,99,8), cv::Scalar(125,255,112) , img_display);
  //imshow(path+"_hsv", img_display);
  //std::vector<cv::Mat> channels;
  //  cv::split(img_hsv, channels);

  //  cv::threshold(img_hsv, img_threshold, 100.0f, 120.0f, 3);  
  //  cv::Mat d;
  //  cv::cvtColor(img_display, d, CV_HSV2BGR);

  cv::Point2f src = tag_location(10);
  std::vector<cv::Vec2f> lines;
  cv::Mat dst, cdst;  
  //  Canny(img_display, dst, 50, 200, 3);
  dst = img_display.clone();
  Canny(img_display, cdst, 50, 200, 3); 

  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(cdst, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point(0, 0) );
  cv::Mat drawing = cv::Mat::zeros(cdst.size(), CV_8UC3 );

  /* //probably don't need because it's taken care of later
     for (int i=1; i<contours.size()-1; i++)
     {
     if (cv::contourArea(contours[i], true) < 0) 
     {
  //remove duplicates
  contours.erase(contours.begin() + i--);
  }
  }*/

  int before = contours.size();
  for (int i=0; i<contours.size(); i++)
  {
    //    std::cout << cv::contourArea(contours[i]) << ", ";
    if (cv::contourArea(contours[i]) > 30) //TODO threshold
    {
      hierarchy.erase(hierarchy.begin() + i);
      contours.erase(contours.begin() + i--);
    }
  }
  std::cout << "Passed contours: " << before - contours.size() << "\n";

  /*
     for(int i=0; i<contours.size(); i++)
     {
     std::stringstream ss;
     ss << "contourr " << i;
     cv::Mat contour1 = cv::Mat::zeros(cdst.size(), CV_8UC3);
     cv::drawContours(contour1, contours, i, cv::Scalar(255,255,255),2,8,hierarchy,0,cv::Point() );
     imshow(ss.str(), contour1);
     }
   */

  std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
  std::vector<cv::Rect> boundRect( contours.size() );
  std::vector<cv::Point2f> center( contours.size() );
  std::vector<float>radius( contours.size() );
  for (int i=0; i<contours.size(); i++)
  {
    if (cv::contourArea(contours[i], true) < 0) //fix counter clockwise
    {
      std::reverse(contours[i].begin(), contours[i].end());
    }
  }
  before = contours.size();
  for (int i=0; i<contours.size(); i++)
  {
    cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
    boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
    cv::minEnclosingCircle( (cv::Mat)contours_poly[i], center[i], radius[i] );
    double tape_circle = 0.5; //TODO threshold
    double rect_obj = (cv::contourArea(contours[i]) / (boundRect[i].width*boundRect[i].height));
  //  std::cout << rect_obj << ", ";
    if (rect_obj > .8 && rect_obj < 1 && cv::contourArea(contours[i]) > 10)
    {
      center.erase(center.begin() + i);
      radius.erase(radius.begin() + i);
      boundRect.erase(boundRect.begin() + i);  
      hierarchy.erase(hierarchy.begin() + i);
      contours.erase(contours.begin() + i--);

    }
    /*    if (cv::contourArea(contours[i])/(PI*radius[i]*radius[i]) < tape_circle)
	  {
    //remove this, probably not tape
    //also remove these if plan to use after
    center.erase(center.begin() + i);
    radius.erase(radius.begin() + i);
    boundRect.erase(boundRect.begin() + i);  
    hierarchy.erase(hierarchy.begin() + i);
    contours.erase(contours.begin() + i--);
    }*/
  }
  std::cout << "Passed contours: " << before - contours.size() << "\n";
  for(int i = 0; i< contours.size(); i++ )
  {
    std::stringstream ss;
    ss << i;
    if (radius[i] < 1) radius[i] = 1.0f;
    radius[i] += 1.5;
    //    cv::drawContours( dst, contours, i, cv::Scalar(0), -1, 8, hierarchy, 0, cv::Point() );
    //  cv::drawContours(drawing, contours, i, cv::Scalar(255,255,255),-1,8,hierarchy,0,cv::Point());
    cv::circle(dst, center[i], radius[i], cv::Scalar(0),-1);
    cv::circle(drawing, center[i], radius[i], cv::Scalar(255,255,255),-1);
    cv::putText(drawing,ss.str() , center[i], cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1,
	cv::Scalar(255,0,0), 3, 3);
  }
  std::cout << "contour count: " << contours.size() << "\n";

  //  imshow(path, dst);

  //imshow("canny", dst);
  //  imshow(path+"_contour", drawing);
  /*
     cv::waitKey(0);*/
  cv::Mat tapes_copy = tapes.clone();
  cv::addWeighted(dst, 0.5, tapes_copy, 0.5, 0.0, tapes);
  publish_image(tapes);

  return;
}



int main(int argc, char **argv)
{

  /*
     test_image("src/img_service/src/img_0.jpg");
     test_image("src/img_service/src/img_1.jpg");
     test_image("src/img_service/src/img_2.jpg");
     imshow("woof", tapes);
     cv::waitKey(0);*/
  //  tapes = NULL;
/*
  for (int i=3; i<11; i++)
  {
    std::stringstream ss;
    ss << "src/img_service/src/img_" << i << ".png";
    test_image(ss.str());
  }
  imshow("woof", tapes);
  cv::waitKey(0);
  return 0;
*/
  ros::init(argc, argv, "img_service_server");
  ros::NodeHandle n;
    image_transport::ImageTransport it(n);
   pub = it.advertise("img_service/image", 1);
  ros::ServiceServer service = n.advertiseService("img_service", detect);
  ros::Subscriber sub = n.subscribe("overhead_camera/image_rect_color", 10, Callback_Img);
  ROS_INFO("Service start");
  ros::spin();

  return 0;
}
