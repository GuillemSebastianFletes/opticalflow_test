#include <trajectory_mono.hpp>

using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace std_msgs;
using namespace message_filters;


// Constructor
trajectory_mono::trajectory_mono()
{
    //first execution variable
    first_execution_ = true;

}

// Destructor
trajectory_mono::~trajectory_mono(){}

// Init
void trajectory_mono::init(){}

void trajectory_mono::calculus(Mat &final_image)
{
    //get the image
    final_image_ = final_image.clone();


    //good features to track variables
    vector<Point2f> NewFeatures;
    int maxCorners = 500;
    double qualityLevel = 0.01;
    double minDistance = 10;
    int blockSize = 3;
    bool useHarrisDetector = false;
    double k = 0.04;

    //optical flows variables
    Mat prev_image;
    vector<Point2f> OldFeatures;
    std::vector<uchar> status;
    std::vector<float> err;

    ///optical flow calculous
    //first the features detection
    goodFeaturesToTrack( final_image_, NewFeatures, maxCorners, qualityLevel, minDistance, Mat(),
                         blockSize, useHarrisDetector, k );



    /*///features detected test
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
      waitKey(100);*/


    /*condition to see if is the first atemp, because to calculate the optical flow
       * the old and the new features are needed*/
    if (first_execution_)
    {
        OldFeatures=NewFeatures;
        prev_image = final_image_.clone();
        first_execution_=false;
    }

    else
    {
        cv::calcOpticalFlowPyrLK(
                    prev_image, final_image_, // 2 consecutive images
                    OldFeatures, // input point positions in first im
                    NewFeatures, // output point positions in the 2nd
                    status,    // tracking success
                    err      // tracking error
                    );


        ///checking the optical flow
        for(size_t i=0; i<NewFeatures.size(); i++)
        {
            if(status[i])
            {
                line(final_image_,OldFeatures[i],NewFeatures[i],Scalar(0,0,255));
            }
        }
        cout<<"Hola"<<endl;
        cv::namedWindow( "optical_flow", CV_WINDOW_AUTOSIZE );
        cv::imshow( "optical_flow", final_image_ );
        waitKey(100);


        OldFeatures=NewFeatures;
        prev_image = final_image_.clone();
    }

}


