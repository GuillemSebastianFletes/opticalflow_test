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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>

using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace std_msgs;


class trajectory_mono
{
    //methods before args.
public:

    trajectory_mono(ros::NodeHandle &nh);
    ~trajectory_mono();
    void show();
    void init();
    void callback(const ImageConstPtr &original, const ImageConstPtr &mask);

    ros::NodeHandle nodehandle_;
    std::string original_image_name_;
    std::string image_mask_name_;

};
