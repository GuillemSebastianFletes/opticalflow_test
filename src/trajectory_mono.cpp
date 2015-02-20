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
    vector<Point2f> NewFeatures, OldFeatures;
    int maxCorners = 500;
    double qualityLevel = 0.01;
    double minDistance = 10;
    int blockSize = 3;
    bool useHarrisDetector = false;
    double k = 0.04;

    //optical flows variables
    std::vector<uchar> status;
    std::vector<float> err;


    //OldFeatures=NewFeatures;

    ///optical flow calculous

    /*condition to see if is the first atemp, because to calculate the optical flow
       * the old and the new features are needed*/
    if (first_execution_)
    {
        cout<<"primera ejecucion"<<endl;
        prev_image_ = final_image_.clone();
        first_execution_=false;

        /*imshow("copia",prev_image);
        imshow("original", final_image_);
        cvWaitKey(100);*/
    }

    else
    {
        //first the features detection
        goodFeaturesToTrack( prev_image_, OldFeatures, maxCorners, qualityLevel, minDistance, Mat(),
                             blockSize, useHarrisDetector, k );

        goodFeaturesToTrack( final_image_, NewFeatures, maxCorners, qualityLevel, minDistance, Mat(),
                             blockSize, useHarrisDetector, k );

        cout<<"** Number of corners detected: "<<NewFeatures.size()<<endl;
        cout<<"** Number of corners detected: "<<OldFeatures.size()<<endl;


        /* ///See what the features vectors have
         for(size_t i=0; i<NewFeatures.size() && i<OldFeatures.size(); i++)
        {
            cout<<NewFeatures[i];
            cout<< "        ";
            cout<<OldFeatures[i]<<endl;
            cvWaitKey(500);
        }*/


        cv::calcOpticalFlowPyrLK(
                    prev_image_, final_image_, // 2 consecutive images
                    OldFeatures, // input point positions in first im
                    NewFeatures, // output point positions in the 2nd
                    status,    // tracking success
                    err      // tracking error
                    );


        ///checking the optical flow
        Mat rgb;
        cvtColor(final_image, rgb, CV_GRAY2BGR);
        int r = 4;
        RNG rng(12345);
        for( int i = 0; i < NewFeatures.size(); i++ )
        {
            cv::circle( rgb, NewFeatures[i], r, Scalar(255,0,0));
        }
        for( int i = 0; i < OldFeatures.size(); i++ )
        {
            cv::circle( rgb, OldFeatures[i], r, Scalar(0,0,255));
        }
        for(size_t i=0; i<NewFeatures.size(); i++)
        {
            if(status[i])
            {
                cv::line(rgb, OldFeatures[i],NewFeatures[i],Scalar(0,255,0));
            }
        }



        cv::namedWindow( "optical_flow", CV_WINDOW_AUTOSIZE );
        cv::imshow( "optical_flow", rgb );
        cv::imshow("actual", final_image_);
        cv::imshow("anterior", prev_image_);
        waitKey(0);


        prev_image_ = final_image_.clone();
        /*imshow("copia",prev_image);
        imshow("original", final_image_);
        cvWaitKey(200);*/

    }

}


