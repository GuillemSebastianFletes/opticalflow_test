#include <trajectory_mono.hpp>

using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace std_msgs;

// Constructor
trajectory_mono::trajectory_mono(ros::NodeHandle &nh)
{
    //rosparam variables initialization
    std::string original_image_name_;
    std::string image_mask_name_;

    //first execution variable
    first_execution_ = true;

    //ROS variables init
    nodehandle_ = nh;
    nodehandle_.getParam("/original_image_name",original_image_name_);
    nodehandle_.getParam("/image_mask_name",image_mask_name_);

    //sincronizer initialization
    message_filters::Subscriber<Image> original_sub(nodehandle_, original_image_name_.data(), 1);
    message_filters::Subscriber<Image> mask_sub(nodehandle_, image_mask_name_.data(), 1);
    message_filters::TimeSynchronizer<Image, Image> sync(original_sub, mask_sub,1);
    sync.registerCallback(boost::bind((&trajectory_mono::callback),this, _1, _2));


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
    Mat final_image_;

    //good features to track variables
    vector<Point2f> NewFeatures;
    int maxCorners = 500;
    double qualityLevel = 0.01;
    double minDistance = 10;
    int blockSize = 3;
    bool useHarrisDetector = false;
    double k = 0.04;

    //optical flows variables
    vector<Point2f> OldFeatures;


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

    ///Showing the images
    cout<< "Testing if the system gets the images";
    cv::imshow("without_mask",original_image_);
    cv::imshow("mask",image_mask_);
    waitKey(300);


    ///aply the mask to the image
    //to avoid errors with the mask aplication we have to initialize the final image before its use
    final_image_= Mat::zeros(cv_disp_ptr->image.rows, cv_disp_ptr->image.cols, CV_8UC1);
    final_image_.copyTo(original_image_,image_mask_);

    ///Testing if the Mask works
    cv::imshow("final_image",final_image_);
    waitKey(300);

    ///optical flow calculous
    //first the features detection
    goodFeaturesToTrack( final_image_, NewFeatures, maxCorners, qualityLevel, minDistance, Mat(),
                         blockSize, useHarrisDetector, k );

    /*features detected test*/
    /// Draw corners detected
      cout<<"** Number of corners detected: "<<NewFeatures.size()<<endl;
      int r = 4;
      RNG rng(12345);
      Mat copy;
      copy = final_image_.clone();
      for( int i = 0; i < NewFeatures.size(); i++ )
         { cv::circle( copy, NewFeatures[i], r, Scalar(rng.uniform(0,255), rng.uniform(0,255),
                  rng.uniform(0,255)), -1, 8, 0 ); }

      /// Show what you got
      cv::namedWindow( "features_detected", CV_WINDOW_AUTOSIZE );
      cv::imshow( "features_detected", copy );

      /*condition to see if is the first atemp, because to calculate the optical flow
       * the old and the new features are needed*/
      if (first_execution_)
      {
          OldFeatures=NewFeatures;
      }

     /* else
      {

      }*/
}




