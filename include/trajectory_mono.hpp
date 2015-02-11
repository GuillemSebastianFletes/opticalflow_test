#include <iostream>
#include <cmath>
#include <math.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/tracking.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>




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
    //void show();
    void init();
    void callback(const ImageConstPtr &original, const ImageConstPtr &mask);

    ros::NodeHandle nodehandle_;
    bool first_execution_;


};
