#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <cmath>
#include <math.h>
#include "opencv2/video/tracking.hpp"
//#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>

using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace std_msgs;


Mat original_image;
Mat mascara_image;


void originalCallback(const ImageConstPtr& disp)
{
    cv_bridge::CvImagePtr cv_disp_ptr;
    try
    {
        cv_disp_ptr = cv_bridge::toCvCopy(disp, "mono8");
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from image to 'mono8'.");
    }

    Mat disparity(cv_disp_ptr->image.rows, cv_disp_ptr->image.cols, CV_8UC1);
//    disparity = cv_disp_ptr->image;
//    original_image = disparity;
    cv::namedWindow("without_mask", CV_WINDOW_AUTOSIZE);
    cv::imshow("without_mask", cv_disp_ptr->image);
    waitKey(300);
}

void maskCallback(const ImageConstPtr& disp1)
{
    cv_bridge::CvImagePtr cv_disp_ptr;
    try
    {
        cv_disp_ptr = cv_bridge::toCvCopy(disp1, "mono8");
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from image to 'mono8'.");
    }

    Mat disparity(cv_disp_ptr->image.rows, cv_disp_ptr->image.cols, CV_8UC1);
//    disparity = cv_disp_ptr->image;
//    mascara_image = disparity;
   cv::namedWindow("mask", CV_WINDOW_AUTOSIZE);
    cv::imshow("mask", cv_disp_ptr->image);
    waitKey(300);
}



int main(int argc, char **argv)
{
    //Opencv variables
    static Mat final_image;

    //Image viewer initialitation
    cv::namedWindow("without_mask", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("mask", CV_WINDOW_AUTOSIZE);
    //cv::namedWindow("with_mask", CV_WINDOW_AUTOSIZE);
    cv::startWindowThread();

    //ROS variables
    ros::init(argc, argv, "optical_flow_test");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber original = it.subscribe("/stereo_camera/disparity", 1, originalCallback);
    image_transport::Subscriber mascara = it.subscribe("/stereo_camera/free_map", 1, maskCallback);

  while (ros::ok())
  {
      original_image.copyTo(final_image, mascara_image);
      cv::imshow("with_mask",final_image);
      loop_rate.sleep();
      ros::spinOnce();

  }
//  cv::destroyWindow("without_mask");
//  cv::destroyWindow("mask");
  //cv::destroyWindow("with_mask");
}
