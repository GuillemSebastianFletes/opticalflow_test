#include <trajectory_mono.hpp>

using namespace message_filters;
using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace std_msgs;


trajectory_mono trajectory_mono_calculus;


void callback(const ImageConstPtr &original, const ImageConstPtr &mask)
{
    //cout<<"I get into the callback"<<endl;
    //Opencv variables initialization
    //Mat final_image;

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

    /*///Showing the images
    //cout<< "Testing if the system gets the images";
    cv::imshow("without_mask",original_image_);
    cv::imshow("mask",image_mask_);
    waitKey(300);*/

    // cout<<"I have the images"<<endl;
    ///aply the mask to the image
    //to avoid errors with the mask aplication we have to initialize the final image before its use
    Mat final_image(cv_disp_ptr->image.rows, cv_disp_ptr->image.cols, CV_8UC1);
//    final_image.zeros(cv_disp_ptr->image.rows, cv_disp_ptr->image.cols, CV_8UC1);
//    original_image_.copyTo(final_image, image_mask_);
    final_image = original_image_.clone();
    //waitKey(10);

 ///Testing if the Mask works
    cv::namedWindow("final_image", CV_WINDOW_AUTOSIZE);
    cv::imshow("final_image",final_image);
    //waitKey(3);*/

   trajectory_mono_calculus.calculus(final_image, image_mask_);
}


int main(int argc, char** argv)
{
    std::string original_image_name_;
    std::string image_mask_name_;

    ros::init(argc, argv, "optical_flow");
    cout<<"starting the program"<<endl;
    ros::NodeHandle nh;
    nh.getParam("/original_image_name",original_image_name_);
    nh.getParam("/image_mask_name",image_mask_name_);
    ros::Publisher translation_pub = nh.advertise<std_msgs::Float64>("translation", 1);
    ros::Publisher rotation_pub = nh.advertise<std_msgs::Float64>("rotation", 1);
    ros::Rate loop_rate(10);
    //cout<<original_image_name_.data()<<endl;
    //cout<<image_mask_name_.data()<<endl;


    //sincronizer initialization
    message_filters::Subscriber<Image> original_sub(nh, original_image_name_.c_str(), 10);
    message_filters::Subscriber<Image> mask_sub(nh, image_mask_name_.c_str(), 10);
    TimeSynchronizer<Image, Image> sync(original_sub, mask_sub, 10);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    cout << "hola" <<endl;
//    loop_rate.sleep();
    ros::spin();
    return 0;
}




