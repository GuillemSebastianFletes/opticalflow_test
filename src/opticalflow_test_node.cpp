#include <trajectory_mono.hpp>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "optical_flow");
    cout<<"starting the program"<<endl;
    ros::NodeHandle nh;
    trajectory_mono trajectory_mono_calculus(nh);
    trajectory_mono_calculus.init();
    ros::spin();
    return 0;
}


