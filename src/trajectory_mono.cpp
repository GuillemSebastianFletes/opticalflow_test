#include <trajectory_mono.hpp>

using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace std_msgs;
using namespace message_filters;


// Constructor
trajectory_mono::trajectory_mono(Mat &final_image)
{
    //first execution variable
    first_execution_ = true;
    //get the image
    final_image_ = final_image;

}

// Destructor
trajectory_mono::~trajectory_mono(){}

// Init
void trajectory_mono::init(){}

void trajectory_mono::calculus()
{
    cout<<"hi"<<endl;
/*
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

    cout<<"I am going to calculus the optical flow"<<endl;
    ///optical flow calculous
    //first the features detection
    goodFeaturesToTrack( final_image_, NewFeatures, maxCorners, qualityLevel, minDistance, Mat(),
                         blockSize, useHarrisDetector, k );

    /*features detected test*/
   /* /// Draw corners detected
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

}


