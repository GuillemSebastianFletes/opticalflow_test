#include <trajectory_mono.hpp>

using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace std_msgs;

// Constructor
trajectory_mono::trajectory_mono(ros::NodeHandle &nh,std::string &original_image, std::string &image_mask)
{
    //ROS variables init
    nodehandle_ = nh;

    //sincronizer initialization
    message_filters::Subscriber<Image> original_sub(nodehandle_, original_image.data(), 1);
    message_filters::Subscriber<Image> mask_sub(nh, image_mask.data(), 1);
    TimeSynchronizer<Image, Image> sync(original_sub, mask_sub,1);
    sync.registerCallback(boost::bind(callback, _1, _2));

}


// Destructor
trajectory_mono::~trajectory_mono(){}

// Init
void trajectory_mono::init(){}

void callback(const ImageConstPtr &original, const ImageConstPtr &mask)
{
    //Windows initialitation
    cv::namedWindow("without_mask", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("mask", CV_WINDOW_AUTOSIZE);

    cv_bridge::CvImagePtr cv_disp_ptr;
    try
    {
        cv_disp_ptr = cv_bridge::toCvCopy(original, "mono8");
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from image to 'mono8'.");
    }

    Mat original_image_(cv_disp_ptr->image.rows, cv_disp_ptr->image.cols, CV_8UC1);
    original_image_=cv_disp_ptr->image;

    try
    {
        cv_disp_ptr = cv_bridge::toCvCopy(mask, "mono8");
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from image to 'mono8'.");
    }

    Mat image_mask_(cv_disp_ptr->image.rows, cv_disp_ptr->image.cols, CV_8UC1);
    image_mask_=cv_disp_ptr->image;

    //Showing the images
    cv::imshow("without_mask",original_image_);
    cv::imshow("mask",image_mask_);
}




