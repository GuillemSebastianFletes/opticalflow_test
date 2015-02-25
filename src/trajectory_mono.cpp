#include <trajectory_mono.hpp>

using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace std_msgs;
using namespace message_filters;

#define PI 3.141592653589793238462643383279502884197169399375105820974944592307816

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


    //translation variables

    Mat new_translation;
    Mat old_translation;

    double angle_translation;
    std::vector<double> translation;
    std::vector<double> repeated_value;
    std::vector<int> number_repetitons;

    double max_value = 0;
    bool elemento_no_presente;
    double x;
    double y;

    /*  //rotation variables
    double x_[];
    double y_[];*/

    //get the image
    final_image_ = final_image.clone();


    ///optical flow calculous

    /*condition to see if is the first atemp, because to calculate the optical flow
       * the old and the new features are needed*/
    if (first_execution_)
    {
        cout<<"primera ejecucion"<<endl;
        prev_image_ = final_image_.clone();
        first_execution_=false;

        /*imshow("copia",prev_image_);
        imshow("original", final_image_);
        cvWaitKey(100);*/
    }

    else
    {
        ///translation calculus

        new_translation = final_image_(Rect(0,2*final_image_.rows/3,final_image_.cols, final_image_.rows/3));
        old_translation = prev_image_(Rect(0,2*final_image_.rows/3,final_image_.cols, final_image_.rows/3));

       //cv::namedWindow( "ROI", CV_WINDOW_AUTOSIZE );
        cv::namedWindow( "old_ROI", CV_WINDOW_AUTOSIZE );
        //imshow("ROI",new_translation);
        imshow("old_ROI", old_translation);
 /*       //cvWaitKey(50);

        //first the features detection
        goodFeaturesToTrack( old_translation, OldFeatures, maxCorners, qualityLevel, minDistance, Mat(),
                             blockSize, useHarrisDetector, k );

        goodFeaturesToTrack( new_translation, NewFeatures, maxCorners, qualityLevel, minDistance, Mat(),
                             blockSize, useHarrisDetector, k );


       /* ///Good features to track check
        cout<<"** Number of corners detected: "<<NewFeatures.size()<<endl;
        cout<<"** Number of corners detected: "<<OldFeatures.size()<<endl;

         ///See what the features vectors have
         for(size_t i=0; i<NewFeatures.size() && i<OldFeatures.size(); i++)
        {
            cout<<NewFeatures[i];
            cout<< "        ";
            cout<<OldFeatures[i]<<endl;
            cvWaitKey(5);
        }*/

  /*      cv::calcOpticalFlowPyrLK(
                    old_translation, new_translation, // 2 consecutive images
                    OldFeatures, // input point positions in first im
                    NewFeatures, // output point positions in the 2nd
                    status,    // tracking success
                    err      // tracking error
                    );

        ///checking the optical flow
        Mat rgb;
        cvtColor(new_translation, rgb, CV_GRAY2BGR);
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
        //cv::imshow("actual", final_image_);
        //cv::imshow("anterior", prev_image_);
        //waitKey(5);


       ///Checking outlaiers
 /*       for( size_t i=0; i < status.size(); i++ )
        {
            if(status[i]) //if there is optical flow get the vector of the desplazament
            {
                x=(NewFeatures[i].x)-(OldFeatures[i].x);
                y=(NewFeatures[i].y)-(OldFeatures[i].y);
                angle_translation = (atan2 (y,x) * 180.0 / PI)*(-1);//angle calculation in degrees

                /*Cheking if the angle belongs to the comfort area*/

 /*               if( angle_translation > 50 && angle_translation < 130 )
                {
                    translation.push_back(sqrt((x*x)+(y*y)));
                }

                else if (angle_translation > 230 && angle_translation < 310 )
                {
                    translation.push_back(sqrt((x*x)+(y*y)));

                }

                else
                {
                    return;

                }
            }
        }


        //Calcule the times that the same value apears
        repeated_value.push_back(translation[0]);
        number_repetitons.push_back(0);

 /*      for ( size_t i=0; i<translation.size(); i++)
        {
            elemento_no_presente = true;
            for (size_t a=0; a<repeated_value.size();a++)
            {
                if (translation[i] == repeated_value[a])
                {
                    number_repetitons[a]++;
                    break;
                    elemento_no_presente = false;

                }
            }

            if (elemento_no_presente)
            {
                repeated_value.push_back(translation[i]);
                number_repetitons.push_back(0);
                cout<<"hola"<<endl;
            }
        }


        /*Get value that apears more
         *  max_value contains the position where the value with the
         * maximum number of repetitions apears*/
/*      for (size_t i= 0; i<number_repetitons.size(); i++ )
        {
            if (number_repetitons[i] >= number_repetitons[max_value])
            {
                max_value = i;
            }
        }

        translation_=repeated_value[max_value];
        cout<<translation_;*/
        prev_image_ = final_image_.clone();
        imshow("copia",prev_image_);
        imshow("original", final_image_);
        cvWaitKey(50);

    }

}


