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


using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace std_msgs;


class trajectory_mono
{
    //methods before args.
private:
    //will subscribe to both images and sincronize them

public:

    trajectory_mono(ros::NodeHandle &nh,std::string &topic1, std::string &topic2);

    ~trajectory_mono();
    void show();
    void init();

    ros::NodeHandle nodehandle_;



};
