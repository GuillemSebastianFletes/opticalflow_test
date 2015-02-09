#include <trajectory_mono.hpp>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "optical flow");
    ros::NodeHandle nh;
    trajectory_mono trajectory_mono_calculus(nh);
    trajectory_mono_calculus.init();
    ros::spin();
    return 0;
}


