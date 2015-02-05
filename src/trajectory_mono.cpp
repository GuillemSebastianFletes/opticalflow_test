#include <trajectory_mono.hpp>

using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace std_msgs;

// Constructor
trajectory_mono::trajectory_mono(ros::NodeHandle &nh)
{
    //ROS variables init
    nodehandle_ = nh;
    original_image__subscriber_ = it.subscribe("/stereo_camera/disparity", 1, originalCallback);
    image_mask_subscriber_ = it_.subscribe("/stereo_camera/free_map", 1, maskCallback);

}


// Destructor
trajectory_mono::~trajectory_mono(){}

// Init
void trajectory_mono::init(){}

void trajectory_mono::originalCallback(const ImageConstPtr &disp)
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

    original_image_(cv_disp_ptr->image.rows, cv_disp_ptr->image.cols, CV_8UC1);
    original_image_ = cv_disp_ptr->image;
}

void trajectory_mono::maskCallback(const ImageConstPtr &disp1)
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

    image_mask_(cv_disp_ptr->image.rows, cv_disp_ptr->image.cols, CV_8UC1);
    image_mask_ = cv_disp_ptr->image;
}
