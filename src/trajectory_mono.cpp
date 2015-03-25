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

void trajectory_mono::translation_calculus(Mat &final_image)
{
    //good features to track variables
    vector<Point2f> NewFeatures, OldFeatures;
    int maxCorners = 1000;
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
    double media;

    double max_value = 0;
    bool elemento_no_presente;
    double x;
    double y;


    /** Create a excel with the data*/

    // pFile=fopen( "translation.csv", "w" );


    ///ROI calculus

    new_translation = final_image(Rect(0,2*final_image.rows/3,final_image.cols, final_image.rows/3));//actual frame
    old_translation = prev_image_(Rect(0,2*final_image.rows/3,final_image.cols, final_image.rows/3));//prev frame

    /** ROI check
    cv::namedWindow( "ROI", CV_WINDOW_AUTOSIZE );
    cv::namedWindow( "old_ROI", CV_WINDOW_AUTOSIZE );
    imshow("ROI",new_translation);
    imshow("old_ROI", old_translation);
    cvWaitKey(50);*/

    /// Features detection

    goodFeaturesToTrack( old_translation, OldFeatures, maxCorners, qualityLevel, minDistance, Mat(),
                         blockSize, useHarrisDetector, k );//prev image

    goodFeaturesToTrack( new_translation, NewFeatures, maxCorners, qualityLevel, minDistance, Mat(),
                         blockSize, useHarrisDetector, k );//actual image

    /** Good features to track check
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

    ///Optical Flow calculus

    cv::calcOpticalFlowPyrLK(
                old_translation, new_translation, // 2 consecutive images
                OldFeatures, // input point positions in first im
                NewFeatures, // output point positions in the 2nd
                status,    // tracking success
                err      // tracking error
                );

    /** checking the optical flow**/
    /*   Mat rgb;
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
    waitKey(1);*/

    /** Checking outlaiers
    *It is know that the module vectors obtained from the optical flow should not be larger than 1/5 * rows of the image.
     * This vector is the result of the join of the same point between frames.
     * At the same time we asume that the angulus of this vector might be higher than 30 but less than 300*/

    for( size_t i=0; i < status.size(); i++ )
    {
        if(status[i]) //if there is optical flow get the vector of the desplazament
        {
            x=(NewFeatures[i].x)-(OldFeatures[i].x);
            y=(NewFeatures[i].y)-(OldFeatures[i].y);
            angle_translation = (atan2 (y,x) * 180.0 / PI)*(-1);//angle calculation in degrees


            /*Cheking if the angle belongs to the comfort area*/
            if(sqrt((x*x)+(y*y)) < final_image.rows/8 && angle_translation > 30 && angle_translation < 150 && (sqrt((x*x)+(y*y))) >1)
            {
                translation.push_back(sqrt((x*x)+(y*y)));

            }

        }
    }
    if (translation.size()==0)
    {
        return;
    }




    /** Calcule the times that the same value apears
    *First of all, repeated_value needs to be initializated to avoid problems with the conditional stamenter of repetion
     * so, the first value of the vector is going to be the first value of the vector that contains the translation values.
     * and number of repetitions of the first value is set to 0
     */

    if (translation.size()>0)
    {
        repeated_value.push_back(translation[0]);
        number_repetitons.push_back(0);

        for ( size_t i=0; i<translation.size(); i++)
        {
            elemento_no_presente = true;
            for (size_t a=0; a<repeated_value.size();a++)
            {
                if (translation[i] == repeated_value[a])
                {
                    number_repetitons[a]++;
                    elemento_no_presente = false;
                    break;

                }
            }

            if (elemento_no_presente)
            {
                repeated_value.push_back(translation[i]);
                number_repetitons.push_back(1);
            }
        }


        /** Get value that apears more
    /*  max_value contains the position where the value with the
     * maximum number of repetitions apears**/
        for (size_t i= 0; i<number_repetitons.size(); i++ )
        {
            if (number_repetitons[i] =  number_repetitons[max_value])
            {
                max_value = i;
            }
        }

        translation_.data = 0;
        media = 0;

        /** Creating an execel to analice the distribution

        for( size_t i=0; i < repeated_value.size(); i++ )
        {
            fprintf (pFile, "%f;", repeated_value[i]);
        }

        fprintf (pFile, "\n");
        fclose (pFile);**/

        for (size_t i= 0; i<number_repetitons.size(); i++ )
        {
            if (number_repetitons[i] >=  0.95*number_repetitons[max_value])
            {
                translation_.data = repeated_value[i] + translation_.data;

                media++;
            }
        }
    }

    translation_.data = translation_.data / media;

}

void trajectory_mono::rotation_calculus(Mat &final_image)
{
    vector<Point2f> NewFeatures, OldFeatures;
    int maxCorners = 1000;
    double qualityLevel = 0.01;
    double minDistance = 10;
    int blockSize = 3;
    bool useHarrisDetector = false;
    double k = 0.04;

    //optical flows variables
    std::vector<uchar> status;
    std::vector<float> err;


    //translation variables
    Mat new_rotation;
    Mat old_rotation;

    double angle_rotation;
    std::vector<double> rotation;
    std::vector<double> repeated_value;
    std::vector<int> number_repetitons;
    double media;

    double max_value = 0;
    bool elemento_no_presente;
    double x;
    double y;


    ///ROI calculus

    new_rotation = final_image(Rect(0,0,final_image.cols, final_image.rows/3));//actual frame
    old_rotation = prev_image_(Rect(0,0,final_image.cols, final_image.rows/3));//prev frame

    /* ///ROI check
    cv::namedWindow( "ROI", CV_WINDOW_AUTOSIZE );
    cv::namedWindow( "old_ROI", CV_WINDOW_AUTOSIZE );
    imshow("ROI",new_translation);
    imshow("old_ROI", old_translation);
    cvWaitKey(5);*/

    ///Features detection
    goodFeaturesToTrack( old_rotation, OldFeatures, maxCorners, qualityLevel, minDistance, Mat(),
                         blockSize, useHarrisDetector, k );//prev image

    goodFeaturesToTrack( new_rotation, NewFeatures, maxCorners, qualityLevel, minDistance, Mat(),
                         blockSize, useHarrisDetector, k );//actual image

    ///Good features to track check
    /*   cout<<"** Number of corners detected: "<<NewFeatures.size()<<endl;
    cout<<"** Number of corners detected: "<<OldFeatures.size()<<endl;

     ///See what the features vectors have
     for(size_t i=0; i<NewFeatures.size() && i<OldFeatures.size(); i++)
    {
        cout<<NewFeatures[i];
        cout<< "        ";
        cout<<OldFeatures[i]<<endl;
        cvWaitKey(5);
    }*/

    ///Optical Flow calculus
    cv::calcOpticalFlowPyrLK(
                old_rotation, new_rotation, // 2 consecutive images
                OldFeatures, // input point positions in first im
                NewFeatures, // output point positions in the 2nd
                status,    // tracking success
                err      // tracking error
                );

    ///checking the optical flow
    /**Mat rgb2;
    cvtColor(new_rotation, rgb2, CV_GRAY2BGR);
    int r = 4;
    RNG rng(12345);
    for( int i = 0; i < NewFeatures.size(); i++ )
    {
        cv::circle( rgb2, NewFeatures[i], r, Scalar(255,0,0));
    }
    for( int i = 0; i < OldFeatures.size(); i++ )
    {
        cv::circle( rgb2, OldFeatures[i], r, Scalar(0,0,255));
    }
    for(size_t i=0; i<NewFeatures.size(); i++)
    {
        if(status[i])
        {
            cv::line(rgb2, OldFeatures[i],NewFeatures[i],Scalar(0,255,0));
        }
    }

    cv::namedWindow( "optical_flow_rotation", CV_WINDOW_AUTOSIZE );
    cv::imshow( "optical_flow", rgb2 );
    waitKey(1);*/

    ///Checking outlaiers
    /*It is know that the module vectors obtained from the optical flow should not be larger than 1/5 * rows of the image.
     * This vector is the result of the join of the same point between frames.
     * At the same time we asume that the angulus of this vector might be higher than 30 but less than 300*/

    for( size_t i=0; i < status.size(); i++ )
    {
        if(status[i]) //if there is optical flow get the vector of the desplazament
        {
            x=(NewFeatures[i].x)-(OldFeatures[i].x);
            y=(NewFeatures[i].y)-(OldFeatures[i].y);
            angle_rotation = (atan2 (y,x) * 180.0 / PI)*(-1);//angle calculation in degrees

            /*Cheking if the angle belongs to the comfort area*/
            if(sqrt((x*x)+(y*y)) < final_image.rows/8 && angle_rotation > 30 && angle_rotation < 340)
            {
                rotation.push_back((atan2 (y,x) * 180.0 / PI)*(-1));
            }

        }
    }
    if (rotation.size()==0)
    {
        return;
    }

    ///Checking rotation vector
    /*  for (size_t i=0; i<rotation.size();i++)
    {
        cout<<"rotation: ";
        cout<<rotation[i]<<endl;
    }*/

    ///Calcule the times that the same value apears
    /*First of all, repeated_value needs to be initializated to avoid problems with the conditional stamenter of repetion
     * so, the first value of the vector is going to be the first value of the vector that contains the translation values.
     * and number of repetitions of the first value is set to 0
     */
    repeated_value.push_back(rotation[0]);
    number_repetitons.push_back(0);

    for ( size_t i=0; i<rotation.size(); i++)
    {
        elemento_no_presente = true;
        for (size_t a=0; a<repeated_value.size();a++)
        {
            if (rotation[i] == repeated_value[a])
            {
                number_repetitons[a]++;
                elemento_no_presente = false;
                break;

            }
        }

        if (elemento_no_presente)
        {
            repeated_value.push_back(rotation[i]);
            number_repetitons.push_back(0);
        }
    }

    ///Get value that apears more
    /*  max_value contains the position where the value with the
     * maximum number of repetitions apears*/
    for (size_t i= 0; i<number_repetitons.size(); i++ )
    {
        if (number_repetitons[i] =  number_repetitons[max_value])
        {
            max_value = i;
        }
    }

    rotation_.data = 0;
    media = 0;

    for (size_t i= 0; i<number_repetitons.size(); i++ )
    {
        if (number_repetitons[i] >=  0.95*number_repetitons[max_value])
        {
            rotation_.data = repeated_value[i] + rotation_.data;
            media++;
        }
    }

    rotation_.data = rotation_.data / media;
    if (rotation_.data > 90)
    {
        rotation_.data = 90 - rotation_.data;
    }

    else
    {
        rotation_.data = 90 - rotation_.data;
    }

    //    cout<<"rotation: ";
    //    cout<<rotation_.data<<endl;
}




void trajectory_mono::calculus(Mat &final_image)
{

    //Publisher initialitation
    ros::NodeHandle n;
    ros::Publisher translation_pub = n.advertise<std_msgs::Float64>("translation", 1);
    ros::Publisher rotation_pub = n.advertise<std_msgs::Float64>("rotation", 1);


    ///optical flow calculous
    /*condition to see if is the first atemp, because to calculate the optical flow
       * the old and the new features are needed*/
    if (first_execution_)
    {
        cout<<"primera ejecucion"<<endl;
        prev_image_ = final_image.clone();
        first_execution_=false;

        /*imshow("copia",prev_image_);
        imshow("original", final_image_);
        cvWaitKey(100);*/
    }

    else
    {
        /* cv::namedWindow("final_image", CV_WINDOW_AUTOSIZE);
        cv::imshow("final_image",final_image);
        waitKey(3);*/
        //cout << "hola" <<endl;
        trajectory_mono::translation_calculus(final_image);
        translation_pub.publish(translation_);
        trajectory_mono::rotation_calculus(final_image);
        rotation_pub.publish(rotation_);

        prev_image_ = final_image.clone();
        //imshow("original", final_image);
        //waitKey(5);
    }
}


