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
    nodehandle_.getParam("/original_image_name",original_image_name_);
    nodehandle_.getParam("/image_mask_name",image_mask_name_);

    //sincronizer initialization
    message_filters::Subscriber<Image> original_sub(nodehandle_, original_image_name_.data(), 1);
    message_filters::Subscriber<Image> mask_sub(nodehandle_, image_mask_name_.data(), 1);
    message_filters::TimeSynchronizer<Image, Image> sync(original_sub, mask_sub,1);
    sync.registerCallback(boost::bind(&trajectory_mono::callback, _1, _2));

    //Windows initialitation to test
    cv::namedWindow("without_mask", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("mask", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("final_image", CV_WINDOW_AUTOSIZE);
}

// Destructor
trajectory_mono::~trajectory_mono(){}

// Init
void trajectory_mono::init(){}

void trajectory_mono::callback(const ImageConstPtr &original, const ImageConstPtr &mask)
{
    //Opencv variables initialization



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
    cout<< "Testing if the system gets the images";
    cv::imshow("without_mask",original_image_);
    cv::imshow("mask",image_mask_);
    waitKey(300);


    //aply the mask to the image
    Mat final_image_ (cv_disp_ptr->image.rows, cv_disp_ptr->image.cols, CV_8UC1);
    //to avoid errors with the mask aplication we have to initialize the final image before its use
    final_image_= Mat::zeros(cv_disp_ptr->image.rows, cv_disp_ptr->image.cols, CV_8UC1);
    final_image_ = Mat::copyTo(original_image_,image_mask_);

    //Testing if the Mask works
    cv::imshow("final_image",final_image_);
    waitKey(300);
}




